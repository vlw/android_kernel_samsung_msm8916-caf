#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Test actual Samsung printk persistence functions with mocked MMIO/locking.

Run under ASan/UBSan. This checks publication, ring semantics and IO ordering,
not ARM mapping attributes or persistence through the Samsung bootloader.
"""
from pathlib import Path
import os
import re
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[4]


def function(source, name):
    match = re.search(r"^[^\n;{}]*\b" + name + r"\([^;{}]*\)\s*\{", source, re.M)
    if not match:
        raise RuntimeError(name)
    begin = match.end() - 1
    depth, end = 1, begin + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


STUBS = r'''
#include <assert.h>
#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef uint32_t u32;
typedef uint64_t u64;
typedef uint64_t phys_addr_t;
#define __iomem
#define __init
#define GFP_KERNEL 0
#define LAST_LOG_BUF_SHIFT 19
#define LOG_MAGIC 0x4d474f4c
#define min(a,b) ((a)<(b)?(a):(b))
#define BUG_ON(x) assert(!(x))
#define pr_info(...) ((void)0)
#define pr_err(...) ((void)0)
#define pr_warn(...) ((void)0)
static u32 *sec_log_ptr;
static char *sec_log_buf;
static unsigned sec_log_size,sec_log_save_size,sec_log_reserve_size;
static phys_addr_t sec_log_save_base,sec_log_reserve_base;
static bool sec_log_reserved;
static unsigned *sec_log_irq_en;
static char *last_kmsg_buffer;
static unsigned last_kmsg_size;
static bool locked,fail_alloc,fail_map,inject_printk;
static unsigned reads,writes,toio_calls,fromio_calls,io_generation,barrier_generation;
static unsigned format_calls;
static int logbuf_lock;
struct log { u32 len,flags; char text[56]; };
static _Alignas(8) char printk_ring[512];
static char *log_buf=printk_ring;
static u32 log_first_idx,log_next_idx;
static u64 log_first_seq,log_next_seq;
static char *mapping;
static void put(unsigned offset,const char *text) {
    struct log *p=(struct log *)(printk_ring+offset);
    assert(offset+sizeof(*p)<=sizeof(printk_ring));
    memset(p,0,sizeof(*p));p->len=sizeof(*p);strcpy(p->text,text);
}
static void lock(void) { assert(!locked);locked=true; }
static void unlock(void) { assert(locked);locked=false; }
#define raw_spin_lock_irqsave(p,f) ((void)(p),(f)=0,lock())
#define raw_spin_unlock_irqrestore(p,f) ((void)(p),(void)(f),unlock())
static u32 readl(const u32 *p) { reads++;return *p; }
static void writel(u32 value,u32 *p) {
    if(sec_log_buf && p==sec_log_ptr) {
        assert(locked && barrier_generation==io_generation);
        writes++;
    }
    *p=value;
}
static void memcpy_toio(void *dst,const void *src,size_t n) {
    assert(locked && sec_log_buf);
    assert((char *)dst>=sec_log_buf && (char *)dst+n<=sec_log_buf+sec_log_size);
    toio_calls++;io_generation++;memcpy(dst,src,n);
}
static void memcpy_fromio(void *dst,const void *src,size_t n) {
    /* The mapping must remain private throughout the old-log snapshot. */
    assert(!locked && !sec_log_buf);fromio_calls++;memcpy(dst,src,n);
}
static void wmb(void) { barrier_generation=io_generation; }
static void *kmalloc(size_t n,int flags) {
    (void)flags;
    assert(!locked && !sec_log_buf);
    if(inject_printk) {
        /* A normal printk arriving while allocation/copying may sleep. */
        put(log_next_idx,"B");log_next_idx+=64;log_next_seq++;
        inject_printk=false;
    }
    return fail_alloc?NULL:malloc(n);
}
static void *ioremap_nocache(phys_addr_t base,size_t size) {
    assert(!sec_log_buf && !locked);
    assert(base==sec_log_save_base-12 && size==sec_log_save_size+12);
    return fail_map?NULL:mapping;
}
static int msg_print_text(const struct log *p,unsigned prev,bool stamp,char *buf,int size) {
    (void)prev;(void)stamp;format_calls++;
    assert(locked && sec_log_buf);
    size_t n=strlen(p->text);assert(n<(unsigned)size);memcpy(buf,p->text,n);return n;
}
static unsigned long long memparse(const char *s,char **end) {
    unsigned long long n=strtoull(s,end,0);
    if(**end=='M') { n<<=20;(*end)++; }
    return n;
}
static unsigned roundup_pow_of_two(unsigned x) {
    unsigned y=1;while(y<x && y<(1U<<31))y<<=1;return y;
}
static int kstrtoull(const char *s,unsigned base,unsigned long long *n) {
    char *end;errno=0;*n=strtoull(s,&end,base);
    return errno || *end || end==s ? -EINVAL : 0;
}
'''

TESTS = r'''
static void reset(unsigned size) {
    free(mapping);free(last_kmsg_buffer);
    mapping=calloc(1,size+12);assert(mapping);
    sec_log_buf=NULL;sec_log_ptr=NULL;sec_log_irq_en=NULL;
    sec_log_size=0;sec_log_save_size=size;sec_log_save_base=0xafe00008;
    last_kmsg_buffer=NULL;last_kmsg_size=0;sec_log_reserved=true;
    locked=fail_alloc=fail_map=inject_printk=false;
    reads=writes=toio_calls=fromio_calls=io_generation=barrier_generation=format_calls=0;
    memset(printk_ring,0,sizeof(printk_ring));
    log_first_idx=log_next_idx=0;log_first_seq=log_next_seq=0;
}
static u32 *position(void) { return (u32 *)(mapping+8); }
static void valid(void) { *(u32 *)(mapping+4)=LOG_MAGIC; }
static void seed(unsigned end) {
    valid();*position()=end;
    unsigned start=end>sec_log_save_size?end-sec_log_save_size:0;
    for(unsigned i=start;i<end;i++)mapping[12+(i&(sec_log_save_size-1))]=(char)(i%251);
}
int main(void) {
    reset(64);
    assert(!sec_log_setup("1M@0xafe00008"));
    assert(sec_log_save_size==(1<<20) && sec_log_save_base==0xafe00008);
    assert(sec_log_setup("1M@0xafe00009")==-EINVAL);
    assert(sec_log_setup("1M@8")==-EINVAL);
    reset(64);
    struct log early={.text="disabled"};sec_log_add(&early);assert(!format_calls);
    fail_map=true;assert(printk_remap_nocache()==-ENOMEM);assert(!sec_log_buf);
    reset(64);sec_log_reserved=false;assert(!printk_remap_nocache());assert(!sec_log_buf);
    /* Allocation may trigger printk, which must only touch its normal ring. */
    reset(64);seed(91);put(0,"A");log_next_idx=64;log_next_seq=1;inject_printk=true;
    assert(!printk_remap_nocache());assert(!locked && sec_log_buf==mapping+12);
    assert(last_kmsg_size==64 && fromio_calls==2);
    for(unsigned i=0;i<64;i++)assert(last_kmsg_buffer[i]==(char)((27+i)%251));
    assert(*position()==93 && format_calls==2 && writes==2);
    assert(mapping[12+(91&63)]=='A' && mapping[12+(92&63)]=='B');
    /* Snapshot allocation failure must not prevent safe live logging. */
    reset(64);seed(50);fail_alloc=true;assert(!printk_remap_nocache());
    assert(sec_log_buf && !last_kmsg_buffer && last_kmsg_size==0);
    /* Previous-boot snapshot is limited by both buffer capacity and 512 KiB. */
    reset(1<<20);seed(800000);assert(!printk_remap_nocache());
    assert(last_kmsg_size==(1<<19));
    for(unsigned i=0;i<last_kmsg_size;i++)
        assert(last_kmsg_buffer[i]==(char)((800000-(1<<19)+i)%251));
    /* printk's physical index has wrapped, while sequence numbers continue. */
    reset(64);put(256,"A");put(320,"B");put(0,"C");put(64,"D");
    log_first_idx=256;log_next_idx=128;log_first_seq=100;log_next_seq=104;
    assert(!printk_remap_nocache());assert(*position()==4);
    assert(!memcmp(mapping+12,"ABCD",4) && format_calls==4);
    /* Random writes, including records larger than the persistent ring. */
    char text[1100],expected[64];
    for(unsigned i=0;i<sizeof(text);i++)text[i]=(char)(i%251);
    unsigned random=0x8916;
    for(unsigned trial=0;trial<5000;trial++) {
        reset(64);assert(!printk_remap_nocache());
        random=random*1664525+1013904223;
        u32 start=trial==0?UINT32_MAX-2:random;
        unsigned count=trial==0?100:trial==1?0:(random>>16)%sizeof(text);
        *position()=start;memset(mapping+12,0x5a,64);memset(expected,0x5a,64);
        for(unsigned i=0;i<count;i++)expected[(start+i)&63]=text[i];
        reads=writes=toio_calls=0;lock();sec_log_write(text,count);unlock();
        u32 end=start+count;
        if(end<start)end=64+(end&63);
        assert(*position()==end);
        assert(!memcmp(mapping+12,expected,64));
        assert(reads==(count?1:0) && writes==(count?1:0) && toio_calls<=2);
        if(trial==0 && count>=64) {
            sec_log_buf=NULL;
            assert(sec_log_save_old(mapping+12,position(),64));
            assert(last_kmsg_size==64);
            assert(!memcmp(last_kmsg_buffer,text+count-64,64));
        }
    }
    free(mapping);free(last_kmsg_buffer);
    puts("PASS: actual sec_log publication and snapshot ordering, OOM/map failures, wrapped printk replay, 5000 ring writes and one ordered position commit per record");
}
'''


def main():
    source = (ROOT / "kernel/printk.c").read_text()
    code = STUBS
    for name in ["log_from_idx", "log_next", "sec_log_write", "sec_log_add",
                 "sec_log_add_on_bootup", "sec_log_save_old", "printk_remap_nocache",
                 "sec_log_setup"]:
        code += "\n" + function(source, name)
    code += "\n" + TESTS
    with tempfile.TemporaryDirectory(prefix="sec-log-test-") as tmp:
        src = Path(tmp) / "test.c"
        binary = Path(tmp) / "test"
        src.write_text(code)
        subprocess.run(shlex.split(os.environ.get("HOSTCC", "cc")) + [
            "-std=gnu11", "-O1", "-g", "-fno-pie", "-no-pie",
            "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
            str(src), "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
