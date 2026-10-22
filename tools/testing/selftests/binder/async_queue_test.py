#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Check the actual BC_FREE_BUFFER async handoff block with host queue stubs.

AOSP binderLibTest.OnewayQueueing remains the required on-device regression
check. This test checks queue selection, FIFO and wakeup, not kernel locking.
"""
from pathlib import Path
import argparse
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[4]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--source", type=Path, default=ROOT / "drivers/staging/android/binder.c")
args = parser.parse_args()
source = args.source.read_text()
start = source.index("if (buffer->async_transaction && buffer->target_node) {")
end = source.index("{", start) + 1
depth = 1
while depth:
    depth += (source[end] == "{") - (source[end] == "}")
    end += 1
block = source[start:end]
code = r'''
#include <assert.h>
#include <stdio.h>
struct list_head { struct list_head *next, *prev; };
static void init(struct list_head *h) { h->next=h; h->prev=h; }
static int list_empty(struct list_head *h) { return h->next == h; }
static void append(struct list_head *n, struct list_head *h) {
    n->prev=h->prev; n->next=h; h->prev->next=n; h->prev=n;
}
static void list_move_tail(struct list_head *n, struct list_head *h) {
    n->prev->next=n->next; n->next->prev=n->prev; append(n,h);
}
struct node { int has_async_transaction; struct list_head async_todo; };
struct buffer { int async_transaction; struct node *target_node; };
struct proc { struct list_head todo; int wait; };
struct thread { struct list_head todo; };
#define BUG_ON(x) assert(!(x))
static void wake_up_interruptible(int *wait) { ++*wait; }
static void release_buffer(struct buffer *buffer, struct proc *proc, struct thread *thread) {
''' + block + r'''
}
int main(void) {
    struct node node = { .has_async_transaction=1 };
    struct buffer buffer = { .async_transaction=1, .target_node=&node };
    struct proc proc = { .wait=0 };
    struct thread thread;
    struct list_head first, second, existing;
    init(&node.async_todo); init(&proc.todo); init(&thread.todo);
    append(&existing,&proc.todo);
    append(&first,&node.async_todo); append(&second,&node.async_todo);
    release_buffer(&buffer,&proc,&thread);
    /* Outbound IPC can consume thread work. Async work must stay off it. */
    assert(list_empty(&thread.todo));
    assert(proc.todo.next==&existing && existing.next==&first);
    assert(node.async_todo.next==&second && node.has_async_transaction);
    assert(proc.wait==1);
    release_buffer(&buffer,&proc,&thread);
    assert(first.next==&second && proc.todo.prev==&second);
    assert(list_empty(&node.async_todo) && node.has_async_transaction);
    assert(proc.wait==2);
    release_buffer(&buffer,&proc,&thread);
    assert(!node.has_async_transaction && proc.wait==2);
    /* Synchronous and node-less buffers must not manipulate async state. */
    node.has_async_transaction=1; buffer.async_transaction=0;
    release_buffer(&buffer,&proc,&thread);
    assert(node.has_async_transaction && proc.wait==2);
    buffer.async_transaction=1; buffer.target_node=0;
    release_buffer(&buffer,&proc,&thread);
    assert(proc.wait==2 && list_empty(&thread.todo));
    puts("PASS: process queue, FIFO, wakeup, empty queue and non-async buffers");
}
'''
with tempfile.TemporaryDirectory(prefix="binder-async-test-") as temp:
    src = Path(temp) / "test.c"
    binary = Path(temp) / "test"
    src.write_text(code)
    subprocess.run([os.environ.get("HOSTCC", "cc"), "-O1", "-g", "-fsanitize=address,undefined",
                    str(src), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True, timeout=10)
