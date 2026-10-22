#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Host regression test for the real memshare request handlers and reservation.

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


STUBS = r"""

#include <assert.h>
#include <errno.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef uint32_t phys_addr_t;
struct device_node {
    const char *label;
    uint32_t size;
    int fail;
};
struct device {
    struct device_node *of_node;
};
struct platform_device {
    struct device dev;
    void *data;
};
#include MEMSHARE_HEADER
#include HEAP_HEADER
#define pr_debug(...) ((void)0)
#define pr_err(...) ((void)0)
#define min(a, b) ((a) < (b) ? (a) : (b))
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define GFP_KERNEL 0
#define PAGE_SIZE 4096
#define DMA_ATTR_NO_KERNEL_MAPPING 1
#define dma_set_attr(bit, ptr) (*(ptr) |= (bit))
struct mutex {
    int held;
};
static void mutex_lock(struct mutex *m) {
    assert(!m->held);
    m->held = 1;
}
static void mutex_unlock(struct mutex *m) {
    assert(m->held);
    m->held = 0;
}
struct driver {
    struct device *dev;
    struct mutex mem_share;
} driver;
struct memshare_child {
    struct device *dev;
};
static struct driver *memsh_drv = &driver;
static struct memshare_child *memsh_child;
static struct mem_blocks memblock[MAX_CLIENTS];
static int client_table[] = {GPS};
static uint32_t num_clients;
static int attrs, mem_share_svc_handle, curr_conn;
static int mem_share_svc_alloc_resp_desc, mem_share_svc_free_resp_desc;
static int mem_share_svc_alloc_generic_resp_desc, mem_share_svc_free_generic_resp_desc;
static struct mem_alloc_resp_msg_v01 allocated;
static struct mem_free_resp_msg_v01 freed;
static struct mem_alloc_generic_resp_msg_v01 generic_allocated;
static struct mem_free_generic_resp_msg_v01 generic_freed;
static unsigned allocations, frees, freed_size, heap_count;
static int fail_allocate, fail_response, send_rc;
static void *managed[32];
static unsigned managed_count;
struct allocation {
    phys_addr_t phy;
    void *cookie;
    unsigned size;
    struct device *dev;
    int active;
};
static struct allocation ledger[128];
static void *kzalloc(size_t n, int flags) {
    void *p;
    if (fail_response)
        return NULL;
    p = calloc(1, n);
    assert(p);
    heap_count++;
    return p;
}
static void kfree(void *p) {
    assert(p && heap_count);
    heap_count--;
    free(p);
}
static void *devm_kzalloc(struct device *d, size_t n, int flags) {
    void *p = calloc(1, n);
    assert(p && managed_count < 32);
    managed[managed_count++] = p;
    return p;
}
static int of_property_read_u32(struct device_node *n, const char *key, uint32_t *v) {
    if (n->fail)
        return -EINVAL;
    *v = n->size;
    return 0;
}
static int of_property_read_string(struct device_node *n, const char *key,
                                   const char **v) {
    if (n->fail)
        return -EINVAL;
    *v = n->label;
    return 0;
}
static void platform_set_drvdata(struct platform_device *d, void *p) { d->data = p; }
static void *dma_alloc_attrs(struct device *d, unsigned n, phys_addr_t *phy, int flags,
                             int *a) {
    struct allocation *b;
    assert(a == &attrs);
    allocations++;
    assert(allocations < 128);
    if (fail_allocate) {
        *phy = UINT_MAX;
        return NULL;
    }
    b = &ledger[allocations];
    b->phy = 0x100000 + allocations * 0x1000;
    b->cookie = (void *)(uintptr_t)b->phy;
    b->size = n;
    b->dev = d;
    b->active = 1;
    *phy = b->phy;
    return b->cookie;
}
static void dma_free_attrs(struct device *d, unsigned n, void *v, phys_addr_t phy,
                           int *a) {
    unsigned i;
    assert(a == &attrs && n);
    for (i = 0; i < 128; i++)
        if (ledger[i].active && ledger[i].phy == phy) {
            assert(ledger[i].cookie == v && ledger[i].size == n && ledger[i].dev == d);
            ledger[i].active = 0;
            frees++;
            freed_size = n;
            return;
        }
    assert(!"Double free or unknown DMA handle");
}
static int qmi_send_resp_from_cb(int h, int c, void *req, int *desc, void *resp,
                                 size_t size) {
    assert(!driver.mem_share.held);
    if (desc == &mem_share_svc_alloc_resp_desc) {
        assert(size == sizeof(allocated));
        memcpy(&allocated, resp, size);
    } else if (desc == &mem_share_svc_free_resp_desc) {
        assert(size == sizeof(freed));
        memcpy(&freed, resp, size);
    } else if (desc == &mem_share_svc_alloc_generic_resp_desc) {
        assert(size == sizeof(generic_allocated));
        memcpy(&generic_allocated, resp, size);
    } else {
        assert(desc == &mem_share_svc_free_generic_resp_desc &&
               size == sizeof(generic_freed));
        memcpy(&generic_freed, resp, size);
    }
    return send_rc;
}
"""

CASES = r"""

static void reset(void) {
    unsigned i;
    assert(!heap_count && !driver.mem_share.held);
    for (i = 0; i < managed_count; i++)
        free(managed[i]);
    managed_count = 0;
    allocations = frees = freed_size = 0;
    fail_allocate = fail_response = send_rc = 0;
    memset(ledger, 0, sizeof(ledger));
    num_clients = 0;
    memsh_child = NULL;
    initialize_client();
}
int main(void) {
    struct mem_alloc_req_msg_v01 a = {.num_bytes = 2048};
    struct mem_free_req_msg_v01 f;
    struct mem_alloc_generic_req_msg_v01 g = {
        .num_bytes = 4096, .client_id = 0, .proc_id = 0, .sequence_id = 42};
    struct mem_free_generic_req_msg_v01 gf = {.client_id_valid = 1, .proc_id_valid = 1};
    struct device_node node = {"modem", 2097152, 0};
    struct platform_device child = {.dev = {.of_node = &node}};
    struct mem_blocks saved;
    unsigned i;
    /* Generic response lifetime, retry, error cleanup and pool exhaustion. */
    reset();
    handle_alloc_generic_req(NULL, &g);
    assert(!heap_count && generic_allocated.resp.result == 0 &&
           generic_allocated.sequence_id == 42);
    assert(generic_allocated.dhms_mem_alloc_addr_info_valid && allocations == 1);
    g.num_bytes = 8192;
    handle_alloc_generic_req(NULL, &g);
    assert(!heap_count && allocations == 1 &&
           generic_allocated.dhms_mem_alloc_addr_info[0].num_bytes == 4096);
    reset();
    for (i = 0; i < MAX_CLIENTS; i++) {
        memblock[i].client_id = i;
        memblock[i].peripheral = 0;
    }
    g.client_id = 55;
    handle_alloc_generic_req(NULL, &g);
    assert(!heap_count && !driver.mem_share.held &&
           generic_allocated.resp.error == QMI_ERR_INVALID_ID_V01);
    assert(!generic_allocated.dhms_mem_alloc_addr_info_valid && !allocations);
    reset();
    g.client_id = 0;
    fail_allocate = 1;
    handle_alloc_generic_req(NULL, &g);
    assert(!heap_count && generic_allocated.resp.result == 1 &&
           !generic_allocated.dhms_mem_alloc_addr_info_valid && !memblock[0].phy_addr);
    fail_allocate = 0;
    handle_alloc_generic_req(NULL, &g);
    assert(generic_allocated.resp.result == 0);
    reset();
    fail_response = 1;
    assert(handle_alloc_generic_req(NULL, &g) == -ENOMEM && !driver.mem_share.held &&
           !heap_count);
    reset();
    g.num_bytes = 0;
    handle_alloc_generic_req(NULL, &g);
    assert(!allocations && generic_allocated.resp.result == 1 && !heap_count);
    reset();
    g.num_bytes = 4096;
    g.client_id = DHMS_MEM_CLIENT_INVALID;
    handle_alloc_generic_req(NULL, &g);
    assert(!allocations && generic_allocated.resp.error == QMI_ERR_INVALID_ID_V01);
    reset();
    g.client_id = 0;
    g.proc_id = 99;
    handle_alloc_generic_req(NULL, &g);
    assert(!allocations && generic_allocated.resp.error == QMI_ERR_INVALID_ID_V01);
    g.proc_id = 0;
    reset();
    send_rc = -EIO;
    assert(handle_alloc_generic_req(NULL, &g) == -EIO && !heap_count &&
           !driver.mem_share.held);
    /* Generic free must match the single actual allocation. */
    reset();
    handle_alloc_generic_req(NULL, &g);
    gf.dhms_mem_alloc_addr_info_len = 1;
    gf.dhms_mem_alloc_addr_info[0].phy_addr = memblock[0].phy_addr;
    gf.dhms_mem_alloc_addr_info[0].num_bytes = 1;
    handle_free_generic_req(NULL, &gf);
    assert(generic_freed.resp.result == 1 && !frees);
    gf.dhms_mem_alloc_addr_info[0].num_bytes = 4096;
    handle_free_generic_req(NULL, &gf);
    assert(generic_freed.resp.result == 0 && frees == 1 && !memblock[0].size);
    handle_free_generic_req(NULL, &gf);
    assert(generic_freed.resp.result == 1 && frees == 1);
    /* Legacy ownership, no duplicate allocation, bounded size and DMA cookie. */
    reset();
    handle_alloc_req(NULL, &a);
    assert(allocations == 1 && memblock[0].size == 2048 && memblock[0].alloted &&
           memblock[0].client_id == 0 && memblock[0].peripheral == 0);
    assert(allocated.resp == 0 && allocated.handle_valid &&
           allocated.handle == memblock[0].phy_addr);
    a.num_bytes = 1024;
    handle_alloc_req(NULL, &a);
    assert(allocations == 1 && allocated.num_bytes == 1024 && memblock[0].size == 2048);
    a.num_bytes = 4096;
    handle_alloc_req(NULL, &a);
    assert(allocations == 1 && allocated.num_bytes == 2048);
    f.handle = memblock[0].phy_addr + 1;
    handle_free_req(NULL, &f);
    assert(freed.resp == 1 && !frees);
    f.handle = memblock[0].phy_addr;
    handle_free_req(NULL, &f);
    assert(freed.resp == 0 && frees == 1 && freed_size == 2048 &&
           !memblock[0].virtual_addr);
    handle_free_req(NULL, &f);
    assert(freed.resp == 1 && frees == 1);
    reset();
    memblock[0].client_id = 2;
    memblock[0].peripheral = 1;
    handle_alloc_req(NULL, &a);
    assert(!allocations && allocated.resp == 1);
    reset();
    fail_allocate = 1;
    handle_alloc_req(NULL, &a);
    assert(allocated.resp == 1 && !allocated.handle_valid && !memblock[0].size &&
           !memblock[0].phy_addr);
    reset();
    a.num_bytes = 0;
    handle_alloc_req(NULL, &a);
    assert(!allocations && allocated.resp == 1);
    a.num_bytes = 2048;
    /* Reservation failures leave the pool untouched; child table has one entry. */
    reset();
    fail_allocate = 1;
    assert(memshare_child_probe(&child) == -ENOMEM && !num_clients && !memblock[0].size &&
           !memblock[0].guarantee && !memblock[0].phy_addr);
    reset();
    node.size = 0;
    assert(memshare_child_probe(&child) == -EINVAL && !allocations && !num_clients);
    node.size = 2097152;
    reset();
    node.label = "unknown";
    assert(memshare_child_probe(&child) == -EINVAL && !allocations);
    node.label = "modem";
    reset();
    assert(!memshare_child_probe(&child));
    saved = memblock[0];
    assert(num_clients == 1 && saved.size == 2097152 && saved.guarantee && saved.alloted);
    assert(memshare_child_probe(&child) == -EINVAL && num_clients == 1 &&
           allocations == 1);
    a.num_bytes = 4194304;
    handle_alloc_req(NULL, &a);
    assert(!allocated.resp && allocated.num_bytes == 2097152 && allocations == 1);
    handle_free_req(NULL, &f);
    assert(!freed.resp && !frees);
    handle_alloc_generic_req(NULL, &g);
    assert(!generic_allocated.resp.result && allocations == 1);
    handle_free_generic_req(NULL, &gf);
    assert(!generic_freed.resp.result && !frees);
    saved = memblock[0];
    free_mem_clients(0);
    assert(!frees && !memcmp(&saved, &memblock[0], sizeof(saved)));
    /* AFTER_POWERUP cleanup releases only dynamic buffers of that peripheral. */
    reset();
    g.proc_id = 0;
    handle_alloc_generic_req(NULL, &g);
    g.proc_id = 1;
    handle_alloc_generic_req(NULL, &g);
    free_mem_clients(0);
    assert(frees == 1 && memblock[0].client_id == DHMS_MEM_CLIENT_INVALID &&
           memblock[1].alloted);
    free_mem_clients(0);
    assert(frees == 1);
    free_mem_clients(1);
    assert(frees == 2);
    reset();
    g.proc_id = 0;
    fail_allocate = 1;
    handle_alloc_generic_req(NULL, &g);
    free_mem_clients(0);
    assert(!frees && memblock[0].client_id == DHMS_MEM_CLIENT_INVALID);
    /* Reject page-alignment overflow before invoking DMA. */
    reset();
    assert(memshare_alloc(NULL, 0, &saved) == -EINVAL &&
           memshare_alloc(NULL, UINT_MAX, &saved) == -EINVAL && !allocations);
    reset();
    puts("PASS: actual memshare handlers, DMA ownership, generic/legacy errors, "
         "reservation rollback, pool exhaustion, SSR cleanup and retained guarantees");
}
"""


def main():
    source = SOURCE.read_text()
    bodies = "\n".join(function(source, name) for name in
                       ("check_client", "free_client", "free_mem_clients",
                        "fill_alloc_response", "initialize_client", "memshare_alloc",
                        "handle_alloc_req", "handle_free_req",
                        "handle_alloc_generic_req", "handle_free_generic_req",
                        "memshare_child_probe"))
    heap = HEADER.parent / "heap_mem_ext_v01.h"
    qmi = (ROOT / "include/soc/qcom/msm_qmi_interface.h").read_text()
    declarations = "#include <stdint.h>\n#include <limits.h>\nstruct elem_info {int unused;};\n"
    for name in ("qmi_result_type_v01", "qmi_error_type_v01", "qmi_response_type_v01"):
        match = re.search(r"(?:enum|struct) " + name + r"\s*\{.*?\};", qmi, re.S)
        declarations += match.group() + "\n"
    code = "#define MEMSHARE_HEADER " + json.dumps(str(HEADER)) + "\n"
    code += "#define HEAP_HEADER " + json.dumps(str(heap)) + "\n"
    code += STUBS + "\n" + bodies + "\n" + CASES
    with tempfile.TemporaryDirectory(prefix="memshare-test-") as directory:
        include = Path(directory) / "soc/qcom"
        include.mkdir(parents=True)
        (include / "msm_qmi_interface.h").write_text(declarations)
        test = Path(directory) / "legacy.c"
        binary = Path(directory) / "legacy-test"
        test.write_text(code)
        compiler = shlex.split(os.environ.get("HOSTCC", "cc"))
        subprocess.run(compiler + ["-std=gnu99", "-fsanitize=address,undefined",
                                   "-I", directory, "-g", str(test), "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
