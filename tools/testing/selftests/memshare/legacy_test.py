#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Host regression test for the real legacy memshare request handlers.

Extract their unchanged C bodies and supply host DMA/QMI test doubles.
This tests ownership and response logic, not DMA hardware or QMI wire encoding.
Run: python3 tools/testing/selftests/memshare/legacy_test.py
"""
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[4]
SOURCE = ROOT / "drivers/soc/qcom/memshare/msm_memshare.c"
HEADER = ROOT / "drivers/soc/qcom/memshare/msm_memshare.h"


def function(source, name):
    match = re.search(r"^(?:static )?(?:int|void) " + name + r"\(", source, re.M)
    if match is None:
        raise RuntimeError("Missing handler: " + name)
    start = source.index("{", match.start())
    depth, end = 1, start + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


STUBS = r'''
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
typedef uintptr_t phys_addr_t;
struct device {int dummy;};
#include MEMSHARE_HEADER
#define DHMS_MEM_CLIENT_INVALID 1000
#define QMI_RESULT_SUCCESS_V01 0
#define QMI_RESULT_FAILURE_V01 1
#define pr_debug(...) ((void)0)
#define pr_err(...) ((void)0)
#define min(a,b) ((a)<(b)?(a):(b))
struct mutex {int held;};
static void mutex_lock(struct mutex *m) {assert(!m->held); m->held=1;}
static void mutex_unlock(struct mutex *m) {assert(m->held);m->held=0;}
struct driver {struct device *dev; struct mutex mem_share;} driver;
static struct driver *memsh_drv=&driver;
static struct mem_blocks memblock[MAX_CLIENTS];
struct mem_alloc_req_msg_v01 {uint32_t num_bytes;};
struct mem_alloc_resp_msg_v01 {int resp;int handle_valid;uint64_t handle;int num_bytes_valid;uint32_t num_bytes;};
struct mem_free_req_msg_v01 {uint32_t handle;};
struct mem_free_resp_msg_v01 {int resp;};
static int attrs, mem_share_svc_handle, curr_conn, mem_share_svc_alloc_resp_desc, mem_share_svc_free_resp_desc;
static struct mem_alloc_resp_msg_v01 allocated;
static struct mem_free_resp_msg_v01 freed;
static unsigned allocations, frees, freed_size;
static int fail_allocate, send_rc;
static int qmi_send_resp_from_cb(int h,int c,void *req,int *desc,void *resp,size_t size) {
 assert(!driver.mem_share.held);
 if(desc==&mem_share_svc_alloc_resp_desc) memcpy(&allocated,resp,size);
 else memcpy(&freed,resp,size);
 return send_rc;
}
int memshare_alloc(struct device *d,unsigned n,struct mem_blocks *b) {
 allocations++;
 if(fail_allocate)return -ENOMEM;
 b->phy_addr=0x100000; b->virtual_addr=(void*)0x4000; return 0;
}
static void dma_free_attrs(struct device *d,unsigned n,void *v,phys_addr_t phy,int *a) {
 assert(a==&attrs && v==(void*)0x4000 && phy==0x100000);
 frees++;freed_size=n;
}

'''

CASES = r'''static void reset(void) {memset(memblock,0,sizeof(memblock));allocations=frees=freed_size=0;fail_allocate=send_rc=0;assert(!driver.mem_share.held);}
int main(void) {
 struct mem_alloc_req_msg_v01 a={2048}; struct mem_free_req_msg_v01 f={0x100001};
 reset();assert(!handle_alloc_req(NULL,&a));assert(allocations==1 && memblock[0].size==2048 && memblock[0].alloted);
 assert(allocated.resp==0 && allocated.handle_valid && allocated.handle==0x100000 && allocated.num_bytes==2048);
 a.num_bytes=1024;handle_alloc_req(NULL,&a);assert(allocations==1 && allocated.num_bytes==1024 && memblock[0].size==2048);
 a.num_bytes=4096;handle_alloc_req(NULL,&a);assert(allocations==1 && allocated.num_bytes==2048);
 handle_free_req(NULL,&f);assert(freed.resp==1 && !frees && memblock[0].size==2048);
 f.handle=0x100000;handle_free_req(NULL,&f);assert(freed.resp==0 && frees==1 && freed_size==2048 && !memblock[0].size && !memblock[0].virtual_addr);
 handle_free_req(NULL,&f);assert(freed.resp==1 && frees==1);
 reset();memblock[0].size=2048;memblock[0].guarantee=memblock[0].alloted=1;memblock[0].phy_addr=0x100000;
 handle_alloc_req(NULL,&a);assert(!allocations && allocated.num_bytes==2048 && allocated.resp==0);
 handle_free_req(NULL,&f);assert(freed.resp==0 && !frees && memblock[0].size==2048 && memblock[0].guarantee);
 reset();fail_allocate=1;handle_alloc_req(NULL,&a);assert(allocated.resp==1 && !allocated.handle_valid && !allocated.num_bytes_valid && !memblock[0].size);
 reset();a.num_bytes=0;handle_alloc_req(NULL,&a);assert(!allocations && allocated.resp==1 && !allocated.handle_valid);
 reset();a.num_bytes=2048;send_rc=-EIO;assert(handle_alloc_req(NULL,&a)==-EIO && !driver.mem_share.held);
 puts("PASS: real legacy handlers: allocation tracking, reuse, bounded reply, failed/zero allocation, handle validation, DMA free/reset, guaranteed retention, transport error");
}
'''


def main():
    source = SOURCE.read_text()
    bodies = "\n".join(function(source, name) for name in
                       ("free_client", "handle_alloc_req", "handle_free_req"))
    code = "#define MEMSHARE_HEADER " + json.dumps(str(HEADER)) + "\n"
    code += STUBS + "\n" + bodies + "\n" + CASES
    with tempfile.TemporaryDirectory(prefix="memshare-test-") as directory:
        test = Path(directory) / "legacy.c"
        binary = Path(directory) / "legacy-test"
        test.write_text(code)
        compiler = shlex.split(os.environ.get("HOSTCC", "cc"))
        subprocess.run(compiler + ["-std=gnu99", "-fsanitize=address,undefined",
                                   "-g", str(test), "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
