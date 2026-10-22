#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""ASan/UBSan test of production ramoops handlers and persistent RAM rings.

Only mapping, allocator, device-tree and ECC operations are host test doubles.
This does not emulate ARM memory attributes, bootloader RAM retention or ECC.
"""
from pathlib import Path
import os
import re
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[4]


def function(source, name):
    match = re.search(r"^[^\n;{}]*\b" + name + r"\(", source, re.M)
    if not match:
        raise RuntimeError(name)
    start = match.start()
    if source[start:].startswith(name + "("):
        start = source.rfind("\n", 0, start - 1) + 1
    begin = source.index("{", match.end())
    depth, end = 1, begin + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def structure(source, name):
    return re.search(r"struct " + name + r" \{.*?^};", source, re.S | re.M)[0]


STUBS = r'''
#define _GNU_SOURCE
#include <assert.h>
#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/types.h>
typedef uint32_t u32;
typedef uint64_t u64;
typedef uint64_t phys_addr_t;
typedef unsigned int uint;
typedef struct { int counter; } atomic_t;
#define atomic_read(p) ((p)->counter)
#define atomic_set(p,v) ((p)->counter = (v))
#define DEFINE_RAW_SPINLOCK(n) int n
static DEFINE_RAW_SPINLOCK(buffer_lock);
#define raw_spin_lock_irqsave(p,f) ((void)(p), (f)=0)
#define raw_spin_unlock_irqrestore(p,f) ((void)(p), (void)(f))
#define notrace
#define GFP_KERNEL 0
#define GFP_ATOMIC 0
#define MIN_MEM_SIZE 4096UL
#define PERSISTENT_RAM_SIG 0x43474244
#define RAMOOPS_KERNMSG_HDR "===="
#define LINUX_VERSION_CODE 199276
#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))
#define unlikely(x) (x)
#define max(a,b) ((a) > (b) ? (a) : (b))
#define pr_info(...) ((void)0)
#define pr_debug(...) ((void)0)
#define pr_err(...) ((void)0)
#define dev_err(...) ((void)0)
#define WARN_ON_ONCE(x) ((void)(x))
#define memcpy_fromio memcpy
#define memcpy_toio memcpy
#define ERR_PTR(n) ((void *)(intptr_t)(n))
#define PTR_ERR(p) ((int)(intptr_t)(p))
#define IS_ERR(p) ((uintptr_t)(p) >= (uintptr_t)-4095)
#define IS_ERR_OR_NULL(p) (!(p) || IS_ERR(p))
enum pstore_type_id { PSTORE_TYPE_DMESG, PSTORE_TYPE_CONSOLE,
    PSTORE_TYPE_FTRACE, PSTORE_TYPE_PMSG };
enum kmsg_dump_reason { KMSG_DUMP_OOPS, KMSG_DUMP_PANIC, KMSG_DUMP_RESTART };
struct pstore_info { void *data, *buf; size_t bufsize; int buf_lock; };
struct property { const char *name; u32 value; };
struct device_node { struct property props[12]; unsigned count; };
struct device { struct device_node *of_node; void *platform_data; };
struct platform_device { struct device dev; };
static unsigned alloc_calls, fail_alloc, live_alloc;
static void *kmalloc(size_t n, int flags) {
    (void)flags;
    if (++alloc_calls == fail_alloc) return NULL;
    void *p = malloc(n ? n : 1); if (p) live_alloc++; return p;
}
static void *kzalloc(size_t n, int flags) {
    void *p=kmalloc(n,flags); if(p) memset(p,0,n); return p;
}
static void kfree(void *p) { if(p) { assert(live_alloc); live_alloc--; free(p); } }
static void *krealloc(void *p, size_t n, int flags) {
    if (!p) return kmalloc(n,flags);
    if (++alloc_calls == fail_alloc) return NULL;
    return realloc(p,n);
}
static void *managed;
static void *devm_kzalloc(struct device *d, size_t n, int f) {
    (void)d; assert(!managed); managed=kzalloc(n,f); return managed;
}
static void *dev_get_platdata(struct device *d) { return d->platform_data; }
static struct property *of_find_property(struct device_node *n,
                                         const char *name, void *len) {
    (void)len;
    for(unsigned i=0;i<n->count;i++)
        if(!strcmp(n->props[i].name,name)) return &n->props[i];
    return NULL;
}
static int of_property_read_u32(struct device_node *n,const char *name,u32 *v) {
    struct property *p=of_find_property(n,name,NULL);
    if(!p) return -EINVAL; *v=p->value; return 0;
}
static bool is_power_of_2(unsigned long x) { return x && !(x&(x-1)); }
static unsigned long rounddown_pow_of_two(unsigned long x) {
    unsigned long y=1; while(x>>=1) y<<=1; return y;
}
static void spin_lock_init(int *p) { *p=0; }
static int __getnstimeofday(struct timespec *t) { memset(t,0,sizeof(*t)); return 0; }
static char *kasprintf(int flags, const char *fmt, ...) {
    (void)flags; char *p; va_list ap; va_start(ap,fmt);
    int r=vasprintf(&p,fmt,ap); va_end(ap); if(r<0) return NULL;
    live_alloc++; return p;
}
static int register_error;
static int pstore_register(struct pstore_info *p) { (void)p; return register_error; }
static unsigned long mem_size,mem_address,record_size;
static unsigned long ramoops_console_size,ramoops_pmsg_size,ramoops_ftrace_size;
static int dump_oops;
'''

CORE_STUBS = r'''
static void persistent_ram_update_ecc(struct persistent_ram_zone *p,unsigned a,unsigned b)
    { (void)p;(void)a;(void)b; }
static void persistent_ram_update_header_ecc(struct persistent_ram_zone *p) { (void)p; }
static void persistent_ram_ecc_old(struct persistent_ram_zone *p) { (void)p; }
static int persistent_ram_init_ecc(struct persistent_ram_zone *p,
                                  struct persistent_ram_ecc_info *e)
    { (void)p;(void)e; return 0; }
static ssize_t persistent_ram_ecc_string(struct persistent_ram_zone *p,char *s,size_t n)
    { (void)p; if(s && n) *s=0; return 0; }
static int mapped, decoder_freed;
static bool pfn_valid(unsigned long pfn) { (void)pfn; return false; }
#define PAGE_SHIFT 12
static void vunmap(void *p) { (void)p; assert(0); }
static void iounmap(void *p) { (void)p; }
static void release_mem_region(phys_addr_t p,size_t n) { (void)p;(void)n; assert(mapped>0);mapped--; }
static void free_rs(struct rs_control *p) { (void)p; decoder_freed++; }
static bool fail_iomap;
static char iomap_area[4096];
static void *request_mem_region(phys_addr_t p,size_t n,const char *name)
    { (void)p;(void)n;(void)name; mapped++;return iomap_area; }
static void *ioremap(phys_addr_t p,size_t n)
    { (void)p;(void)n;return fail_iomap ? NULL : iomap_area; }
#define ioremap_wc ioremap
'''

NEW_STUB = r'''
#define BASE 0x9ff00000UL
#define TOTAL 0x100000UL
static unsigned char ram[TOTAL];
static int zone_calls, fail_zone;
static struct persistent_ram_zone *persistent_ram_new(phys_addr_t start,size_t size,
        u32 sig,struct persistent_ram_ecc_info *ecc,unsigned memtype) {
    (void)memtype;
    if(++zone_calls==fail_zone) return ERR_PTR(-ENOMEM);
    assert(start>=BASE && start-BASE+size<=TOTAL);
    struct persistent_ram_zone *p=kzalloc(sizeof(*p),GFP_KERNEL);
    if(!p) return ERR_PTR(-ENOMEM);
    p->paddr=start;p->size=size;p->vaddr=ram+(start-BASE);p->buffer=p->vaddr;
    p->buffer_size=size-sizeof(struct persistent_ram_buffer);mapped++;
    int ret=persistent_ram_post_init(p,sig,ecc);
    if(ret) { persistent_ram_free(p); return ERR_PTR(ret); }
    return p;
}
'''

TESTS = r'''
static struct ramoops_platform_data config(void) {
    struct ramoops_platform_data p={0};
    p.mem_address=BASE;p.mem_size=TOTAL;p.record_size=0x40000;
    p.console_size=0x80000;p.pmsg_size=0x40000;p.dump_oops=1;
    return p;
}
static void cleanup(void) {
    if(oops_cxt.size) {
        kfree(oops_cxt.pstore.buf);
        persistent_ram_free(oops_cxt.cprz);
        persistent_ram_free(oops_cxt.fprz);
        persistent_ram_free(oops_cxt.mprz);
        ramoops_free_przs(&oops_cxt);
    }
    kfree(managed);managed=NULL;
    memset(&oops_cxt,0,sizeof(oops_cxt));
    assert(!mapped && !live_alloc);
    fail_alloc=0;alloc_calls=0;fail_zone=0;zone_calls=0;register_error=0;
}
int main(void) {
    fail_iomap=true;
    assert(!persistent_ram_iomap(BASE,4096,0));assert(!mapped);
    fail_iomap=false;
    assert(persistent_ram_iomap(BASE,4096,0)==iomap_area);
    release_mem_region(BASE,4096);assert(!mapped);
    struct ramoops_platform_data p=config(),bad=p;
    assert(!ramoops_validate_layout(&p));
    bad.console_size=ULONG_MAX;assert(ramoops_validate_layout(&bad)==-EINVAL);
    bad=p;bad.mem_address=ULONG_MAX-TOTAL+1;assert(ramoops_validate_layout(&bad)==-EINVAL);
    bad=p;bad.record_size=TOTAL;assert(ramoops_validate_layout(&bad)==-EINVAL);
    bad=p;bad.record_size=1;assert(ramoops_validate_layout(&bad)==-EINVAL);
    bad=p;bad.ecc_info.ecc_size=INT_MAX;assert(ramoops_validate_layout(&bad)==-EINVAL);
    struct platform_device dev={.dev={.platform_data=&p}};
    for(int i=1;i<=3;i++) {
        fail_zone=i;assert(ramoops_probe(&dev)==-ENOMEM);cleanup();
    }
    register_error=-EBUSY;assert(ramoops_probe(&dev)==-EBUSY);cleanup();
    for(unsigned i=1;i<=12;i++) {
        fail_alloc=i;int ret=ramoops_probe(&dev);assert(ret==0 || ret==-ENOMEM);cleanup();
    }
    memset(ram,0,sizeof(ram));assert(!ramoops_probe(&dev));
    assert(ramoops_probe(&dev)==-EBUSY);
    u64 id=0;char large[10000];memset(large,'L',sizeof(large));
    assert(!ramoops_pstore_write_buf(PSTORE_TYPE_DMESG,KMSG_DUMP_PANIC,&id,1,
                                    large,sizeof(large),&oops_cxt.pstore));
    persistent_ram_save_old(oops_cxt.przs[0]);
    assert(!ramoops_pstore_write_buf(PSTORE_TYPE_DMESG,KMSG_DUMP_OOPS,&id,1,
                                    "short",5,&oops_cxt.pstore));
    persistent_ram_save_old(oops_cxt.przs[0]);
    assert(persistent_ram_old_size(oops_cxt.przs[0])<100);
    /* A later larger snapshot must resize the old_log allocation. */
    assert(!ramoops_pstore_write_buf(PSTORE_TYPE_DMESG,KMSG_DUMP_PANIC,&id,1,
                                    large,sizeof(large),&oops_cxt.pstore));
    persistent_ram_save_old(oops_cxt.przs[0]);
    assert(persistent_ram_old_size(oops_cxt.przs[0])>sizeof(large));
    assert(!ramoops_pstore_write_buf(PSTORE_TYPE_PMSG,0,&id,0,"PMSG_MARKER",11,&oops_cxt.pstore));
    assert(!ramoops_pstore_write_buf(PSTORE_TYPE_CONSOLE,0,&id,0,"CONSOLE_MARKER",14,&oops_cxt.pstore));
    cleanup(); /* Simulate a reboot: free metadata, keep physical RAM. */
    assert(!ramoops_probe(&dev));
    assert(persistent_ram_old_size(oops_cxt.cprz)==14);
    assert(!memcmp(persistent_ram_old(oops_cxt.cprz),"CONSOLE_MARKER",14));
    assert(persistent_ram_old_size(oops_cxt.mprz)==11);
    assert(!memcmp(persistent_ram_old(oops_cxt.mprz),"PMSG_MARKER",11));
    assert(ramoops_pstore_erase(PSTORE_TYPE_FTRACE,0,0,(struct timespec){0},&oops_cxt.pstore)==-EINVAL);
    cleanup();
    /* Empty record zero must not hide populated record one. */
    memset(ram,0,sizeof(ram));p.record_size=0x20000;
    assert(!ramoops_probe(&dev));
    persistent_ram_write(oops_cxt.przs[1],"SECOND",6);
    ramoops_pstore_open(&oops_cxt.pstore);
    enum pstore_type_id type;int count=-1;struct timespec time;char *buf;
    assert(ramoops_pstore_read(&id,&type,&count,&time,&buf,&oops_cxt.pstore)==6);
    assert(id==1 && type==PSTORE_TYPE_DMESG && count==0 && !memcmp(buf,"SECOND",6));
    kfree(buf);
    /* Ring wrapping preserves the newest bytes in chronological order. */
    unsigned capacity=oops_cxt.cprz->buffer_size;
    char *pattern=malloc(capacity+123);assert(pattern);
    for(unsigned i=0;i<capacity+123;i++)pattern[i]=(char)(i%251);
    persistent_ram_write(oops_cxt.cprz,pattern,capacity+123);
    persistent_ram_save_old(oops_cxt.cprz);
    assert(persistent_ram_old_size(oops_cxt.cprz)==capacity);
    assert(!memcmp(persistent_ram_old(oops_cxt.cprz),pattern+123,capacity));
    assert(!ramoops_pstore_write_buf(PSTORE_TYPE_DMESG,KMSG_DUMP_PANIC,&id,1,
                                    pattern,capacity+123,&oops_cxt.pstore));
    persistent_ram_save_old(oops_cxt.przs[0]);
    unsigned crash_capacity=oops_cxt.przs[0]->buffer_size;
    unsigned header_size=strlen("====0.0\n");
    assert(persistent_ram_old_size(oops_cxt.przs[0])==crash_capacity);
    assert(!memcmp((char *)persistent_ram_old(oops_cxt.przs[0])+header_size,
                   pattern+capacity+123-(crash_capacity-header_size),
                   crash_capacity-header_size));
    free(pattern);
    oops_cxt.cprz->rs_decoder=(struct rs_control *)1;
    cleanup();assert(decoder_freed==1);
    /* DT failures must fail the probe instead of claiming a working backend. */
    struct device_node node={0};dev.dev.of_node=&node;
    assert(ramoops_probe(&dev)==-EINVAL);cleanup();
    node.props[node.count++]=(struct property){"android,ramoops-buffer-start",BASE};
    node.props[node.count++]=(struct property){"android,ramoops-buffer-size",TOTAL};
    node.props[node.count++]=(struct property){"android,ramoops-console-size",0x80000};
    node.props[node.count++]=(struct property){"android,ramoops-pmsg-size",0x40000};
    node.props[node.count++]=(struct property){"android,ramoops-record-size",0x40000};
    node.props[node.count++]=(struct property){"android,ramoops-ecc-size",16};
    assert(!ramoops_probe(&dev));assert(oops_cxt.ecc_info.ecc_size==16);cleanup();
    puts("PASS: actual ramoops probe, layout bounds, failure cleanup, record replacement, sparse reads, RAM ring wrap and simulated reboot recovery (ECC mocked)");
}
'''


def main():
    ram = (ROOT / "fs/pstore/ram.c").read_text()
    core = (ROOT / "fs/pstore/ram_core.c").read_text()
    header = (ROOT / "include/linux/pstore_ram.h").read_text()
    code = STUBS + "\n" + "\n".join(structure(header, name) for name in [
        "persistent_ram_ecc_info", "persistent_ram_zone", "ramoops_platform_data"])
    code += "\n" + structure(core, "persistent_ram_buffer")
    code += "\n" + structure(ram, "ramoops_context")
    code += "\nstatic struct ramoops_context oops_cxt;\n" + CORE_STUBS
    for name in ["buffer_size", "buffer_start", "buffer_start_add", "buffer_size_add",
                 "persistent_ram_update", "persistent_ram_save_old", "persistent_ram_write",
                 "persistent_ram_old_size", "persistent_ram_old", "persistent_ram_free_old",
                 "persistent_ram_zap", "persistent_ram_post_init", "persistent_ram_free",
                 "persistent_ram_iomap"]:
        code += "\n" + function(core, name)
    code += "\n" + NEW_STUB
    # Select the CONFIG_OF implementation, not its !CONFIG_OF stub.
    of = ram[ram.index("#ifdef CONFIG_OF"):ram.index("#else", ram.index("#ifdef CONFIG_OF"))]
    code += "\n" + function(of, "ramoops_of_init")
    for name in ["ramoops_pstore_open", "ramoops_get_next_prz", "prz_ok", "ramoops_pstore_read",
                 "ramoops_write_kmsg_hdr", "ramoops_pstore_write_buf", "ramoops_pstore_erase",
                 "ramoops_free_przs", "ramoops_init_przs", "ramoops_init_prz",
                 "ramoops_validate_layout", "ramoops_probe"]:
        code += "\n" + function(ram, name)
    code += "\n" + TESTS
    with tempfile.TemporaryDirectory(prefix="ramoops-test-") as tmp:
        src = Path(tmp) / "test.c"
        binary = Path(tmp) / "test"
        src.write_text(code)
        subprocess.run(shlex.split(os.environ.get("HOSTCC", "cc")) + [
            "-std=gnu11", "-g", "-O1", "-fno-pie", "-no-pie",
            "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
            str(src), "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
