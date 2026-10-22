#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Exercise production Binder priority helpers using a scheduler/capability shim.

Checks policy decisions, limits, nesting and restoration, not live scheduling.
"""
from pathlib import Path
import os
import re
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[4]


def function(source, name):
    match = re.search(r"^static (?:bool|int|void) " + name + r"\(", source, re.M)
    assert match, name
    start = source.index("{", match.start())
    depth, end = 1, start + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


SHIM = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#define SCHED_NORMAL 0
#define SCHED_FIFO 1
#define SCHED_RR 2
#define SCHED_BATCH 3
#define SCHED_IDLE 5
#define SCHED_RESET_ON_FORK 0x40000000
#define DEFAULT_PRIO 120
#define MAX_USER_RT_PRIO 100
#define CAP_SYS_NICE 23
#define RLIMIT_RTPRIO 0
#define RLIMIT_NICE 1
#define BINDER_DEBUG_PRIORITY_CAP 0
#define TF_ONE_WAY 1
#define binder_debug(...) ((void)0)
#define binder_user_error(...) ((void)0)
#define trace_binder_set_priority(...) ((void)0)
struct binder_priority { unsigned sched_policy; int prio; };
struct task_struct { unsigned policy; int normal_prio, static_prio, tgid, pid;
    bool cap; unsigned long limits[2]; };
struct binder_proc { struct binder_priority default_priority; };
struct binder_transaction { struct binder_priority priority, saved_priority;
    bool set_priority_called; unsigned flags; };
struct sched_param { int sched_priority; };
static bool has_capability_noaudit(struct task_struct *t, int c) {
    assert(c == CAP_SYS_NICE); return t->cap;
}
static unsigned long task_rlimit(struct task_struct *t, int l) { return t->limits[l]; }
static unsigned scheduler_calls;
static int sched_setscheduler_nocheck(struct task_struct *t, unsigned policy,
                                      struct sched_param *p) {
    assert(policy & SCHED_RESET_ON_FORK);
    t->policy = policy & ~SCHED_RESET_ON_FORK;
    bool rt = t->policy == SCHED_FIFO || t->policy == SCHED_RR;
    assert(rt ? p->sched_priority >= 1 && p->sched_priority <= 99 : p->sched_priority == 0);
    t->normal_prio = rt ? 99 - p->sched_priority : t->static_prio;
    scheduler_calls++; return 0;
}
static void set_user_nice(struct task_struct *t, int nice) {
    assert(nice >= -20 && nice <= 19);
    t->static_prio = 120 + nice;
    if (t->policy == SCHED_NORMAL || t->policy == SCHED_BATCH)
        t->normal_prio = t->static_prio;
}
'''

TESTS = r'''
static struct task_struct task(bool cap, unsigned long rt, unsigned long nice) {
    return (struct task_struct){.normal_prio=120, .static_prio=120,
        .cap=cap, .limits={rt,nice}};
}
static struct binder_priority priority(unsigned policy, int user) {
    return (struct binder_priority){policy,to_kernel_prio(policy,user)};
}
static void check(struct task_struct *t, unsigned policy, int user) {
    assert(t->policy == policy && to_userspace_prio(policy,t->normal_prio)==user);
}
int main(void) {
    for (int policy=0; policy<=3; policy++) {
        assert(binder_supported_policy(policy));
        for (int u=(policy==1||policy==2)?1:-20;
             u<=((policy==1||policy==2)?99:19); u++)
            assert(to_userspace_prio(policy,to_kernel_prio(policy,u))==u);
    }
    assert(!binder_supported_policy(SCHED_IDLE));
    struct task_struct receiver=task(true,0,0);
    struct binder_priority normal=priority(SCHED_NORMAL,0), rr7=priority(SCHED_RR,7);
    struct binder_transaction t={.priority=normal};
    binder_transaction_priority(&receiver,&t,rr7,false);
    check(&receiver,SCHED_RR,7); /* Explicit node RT policy works without inheritance. */
    assert(t.saved_priority.sched_policy==SCHED_NORMAL && t.saved_priority.prio==120);
    unsigned calls=scheduler_calls;
    binder_transaction_priority(&receiver,&t,normal,false);
    check(&receiver,SCHED_RR,7);assert(scheduler_calls==calls);
    struct binder_transaction nested={.priority=priority(SCHED_RR,8)};
    binder_transaction_priority(&receiver,&nested,rr7,true);
    check(&receiver,SCHED_RR,8);
    binder_restore_priority(&receiver,nested.saved_priority);check(&receiver,SCHED_RR,7);
    binder_restore_priority(&receiver,t.saved_priority);check(&receiver,SCHED_NORMAL,0);
    t=(struct binder_transaction){.priority=priority(SCHED_RR,8)};
    binder_transaction_priority(&receiver,&t,priority(SCHED_NORMAL,-10),false);
    check(&receiver,SCHED_NORMAL,-10); /* Node floor survives disabling caller RT. */
    receiver=task(true,0,0);
    t=(struct binder_transaction){.priority=priority(SCHED_RR,7)};
    binder_transaction_priority(&receiver,&t,priority(SCHED_FIFO,7),true);
    check(&receiver,SCHED_FIFO,7);
    receiver=task(false,5,40);
    binder_set_priority(&receiver,priority(SCHED_RR,8));check(&receiver,SCHED_RR,5);
    receiver=task(false,0,25);
    binder_set_priority(&receiver,priority(SCHED_FIFO,8));check(&receiver,SCHED_NORMAL,-5);
    receiver=task(false,0,20);
    binder_set_priority(&receiver,priority(SCHED_BATCH,-10));check(&receiver,SCHED_BATCH,0);
    receiver=task(false,0,0);
    binder_set_priority(&receiver,priority(SCHED_NORMAL,-10));check(&receiver,SCHED_NORMAL,0);
    binder_restore_priority(&receiver,rr7);check(&receiver,SCHED_RR,7);
    binder_restore_priority(&receiver,normal);check(&receiver,SCHED_NORMAL,0);
    struct task_struct caller=task(true,0,0);
    caller.policy=SCHED_RR;caller.normal_prio=91;
    struct binder_proc proc={.default_priority=priority(SCHED_BATCH,3)};
    t=(struct binder_transaction){0};capture(&caller,&proc,&t);
    assert(t.priority.sched_policy==SCHED_RR && t.priority.prio==91);
    t.flags=TF_ONE_WAY;capture(&caller,&proc,&t);
    assert(t.priority.sched_policy==SCHED_BATCH && t.priority.prio==123);
    caller.policy=SCHED_IDLE;t.flags=0;capture(&caller,&proc,&t);
    assert(t.priority.sched_policy==SCHED_BATCH && t.priority.prio==123);
    puts("PASS: production RT/node policy, opt-in inheritance, FIFO tie, limits, nesting, restore and async fallback");
}
'''


def main():
    source = (ROOT / "drivers/staging/android/binder.c").read_text()
    # Kernel integration invariants: default context-manager nodes are initialized,
    # task references are balanced and the old nice-only path is gone.
    new_node = source[source.index("static struct binder_node *binder_new_node("):
                      source.index("static int binder_inc_node(")]
    assert "node->min_priority = to_kernel_prio(" in new_node
    assert "s8 priority = flags & FLAT_BINDER_FLAG_PRIORITY_MASK;" in new_node
    assert source.count("get_task_struct(current);") == 1
    assert source.count("put_task_struct(thread->task);") == 1
    assert "binder_set_nice(" not in source
    start = source.index("\tif (!(t->flags & TF_ONE_WAY) &&\n")
    end = source.index("\n\n\tif (target_node", start)
    capture = ("static void capture(struct task_struct *current, struct binder_proc *target_proc, "
               "struct binder_transaction *t) {\n" + source[start:end] + "\n}\n")
    code = SHIM + "\n" + "\n".join(function(source, name) for name in [
        "is_rt_policy", "is_fair_policy", "binder_supported_policy", "to_userspace_prio",
        "to_kernel_prio", "binder_do_set_priority", "binder_set_priority",
        "binder_restore_priority", "binder_transaction_priority"]) + "\n" + capture + TESTS
    with tempfile.TemporaryDirectory(prefix="binder-priority-") as tmp:
        src=Path(tmp)/"test.c";exe=Path(tmp)/"test";src.write_text(code)
        subprocess.run(shlex.split(os.environ.get("HOSTCC", "cc")) + [
            "-std=gnu11", "-O1", "-g", "-fno-pie", "-no-pie", "-Wall", "-Werror",
            "-fsanitize=address,undefined", "-fno-omit-frame-pointer", str(src), "-o", str(exe)],check=True)
        subprocess.run([str(exe)],check=True)


if __name__ == "__main__":
    main()
