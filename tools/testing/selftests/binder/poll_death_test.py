#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Exercise production Binder poll/death queue code with host waitqueue stubs."""
from pathlib import Path
import argparse
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[4]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--source", type=Path, default=ROOT / "drivers/staging/android/binder.c")
parser.add_argument("--mode", choices=["poll", "death", "both"], default="both")
args = parser.parse_args()
source = args.source.read_text()


def block(start):
    begin = source.index(start)
    end = source.index("{", begin) + 1
    depth = 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[begin:end]


poll = block("static unsigned int binder_poll(")
death = block("if (ref->node->proc == NULL) {")
code = r'''
#include <assert.h>
#include <stdio.h>
#include <stddef.h>
struct list_head { struct list_head *next, *prev; };
static void init(struct list_head *h) { h->next=h; h->prev=h; }
static int list_empty(struct list_head *h) { return h->next == h; }
static void list_add_tail(struct list_head *n, struct list_head *h) {
    n->prev=h->prev; n->next=h; h->prev->next=n; h->prev=n;
}
struct waitqueue { int registrations, wakes; };
struct binder_proc { struct list_head todo; struct waitqueue wait; };
struct binder_thread {
    struct list_head todo; struct waitqueue wait;
    void *transaction_stack; int return_error, looper;
};
struct file { void *private_data; };
struct poll_table_struct { int can_register; };
struct work { int type; struct list_head entry; };
struct death { struct work work; };
struct node { struct binder_proc *proc; };
struct ref { struct node *node; struct death *death; };
#define BR_OK 0
#define POLLIN 1
#define BINDER_LOOPER_STATE_REGISTERED 1
#define BINDER_LOOPER_STATE_ENTERED 2
#define BINDER_WORK_DEAD_BINDER 9
static struct binder_thread test_thread;
static int proc_ready, thread_ready;
#define binder_lock(tag) do {} while (0)
#define binder_unlock(tag) do {} while (0)
static struct binder_thread *binder_get_thread(struct binder_proc *p) { return &test_thread; }
static int binder_has_proc_work(struct binder_proc *p, struct binder_thread *t) { return proc_ready; }
static int binder_has_thread_work(struct binder_thread *t) { return thread_ready; }
static void poll_wait(struct file *f, struct waitqueue *q, struct poll_table_struct *p) {
    if (p->can_register) q->registrations++;
}
static void wake_up_interruptible(struct waitqueue *q) { q->wakes++; }
''' + poll + r'''
static void request_death(struct ref *ref, struct binder_proc *proc, struct binder_thread *thread) {
''' + death + r'''
}
static void test_poll(void) {
    struct binder_proc proc = {0};
    struct file file = { .private_data=&proc };
    struct poll_table_struct poll = { .can_register=1 };
    init(&proc.todo); init(&test_thread.todo);
    proc_ready=1; thread_ready=0;
    assert(binder_poll(&file,&poll)==POLLIN);
    /* epoll may only offer registration on its first poll invocation. */
    assert(proc.wait.registrations==1);
    poll.can_register=0; proc_ready=0;
    assert(binder_poll(&file,&poll)==0);
    assert(proc.wait.registrations==1);
    /* Also cover the thread-specific workqueue branch. */
    poll.can_register=1; thread_ready=1;
    test_thread.transaction_stack=&file;
    assert(binder_poll(&file,&poll)==POLLIN);
    assert(test_thread.wait.registrations==1);
    poll.can_register=0; thread_ready=0;
    assert(binder_poll(&file,&poll)==0);
    assert(test_thread.wait.registrations==1);
    puts("PASS: initial readiness still registers epoll wakeups on both queues");
}
static void test_death(void) {
    for (int looper=0;looper<4;looper++) {
        struct binder_proc proc = {0};
        struct binder_thread thread = { .looper=looper };
        struct node node = { .proc=NULL };
        struct death death = {0};
        struct ref ref = { .node=&node, .death=&death };
        init(&proc.todo); init(&thread.todo);
        request_death(&ref,&proc,&thread);
        assert(proc.todo.next==&death.work.entry);
        assert(list_empty(&thread.todo));
        assert(proc.wait.wakes==1 && death.work.type==BINDER_WORK_DEAD_BINDER);
        init(&proc.todo); proc.wait.wakes=0; node.proc=&proc;
        request_death(&ref,&proc,&thread);
        assert(list_empty(&proc.todo) && proc.wait.wakes==0);
    }
    puts("PASS: death callbacks use process workqueue for all looper states");
}
int main(void) {
''' + ("test_poll();" if args.mode in ("poll", "both") else "") + (
    "test_death();" if args.mode in ("death", "both") else "") + "\n}\n"
with tempfile.TemporaryDirectory(prefix="binder-poll-death-test-") as temp:
    src = Path(temp) / "test.c"
    binary = Path(temp) / "test"
    src.write_text(code)
    subprocess.run([os.environ.get("HOSTCC", "cc"), "-O1", "-g", "-fsanitize=address,undefined",
                    str(src), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True, timeout=10)
