#!/usr/bin/env python3
"""Exercise the actual A3 input wake callbacks with a pthread workqueue shim.

Hardware power/I2C operations are stubs. This checks cancellation and PM
serialization, not controller timing or framebuffer hardware behaviour.
"""
from pathlib import Path
import os
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[4]


def function(source, signature):
    start = source.index(signature + "\n{")
    brace = source.index("{", start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


SHIM = r'''
#include <assert.h>
#include <pthread.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#define CONFIG_FB 1
#define pr_info(...) do {} while (0)
#define container_of(p, type, member) ((type *)((char *)(p) - offsetof(type, member)))
#define mutex_lock(p) pthread_mutex_lock(p)
#define mutex_unlock(p) pthread_mutex_unlock(p)
#define gpio_direction_input(p) 0
#define gpio_set_value(p, v) do {} while (0)
#define FB_EVENT_BLANK 9
#define FB_BLANK_UNBLANK 0
#define FB_BLANK_NORMAL 1
#define FB_BLANK_VSYNC_SUSPEND 2
#define FB_BLANK_HSYNC_SUSPEND 3
#define FB_BLANK_POWERDOWN 4
struct notifier_block { int unused; };
struct fb_event { void *data; };
struct work_struct {
    pthread_mutex_t lock;
    pthread_cond_t done;
    bool pending, running;
};
struct device { int unused; };
struct i2c_client { struct device dev; };
struct input_dev { void *info; };
struct pdata { int gpio_scl, gpio_sda; };
struct INFO {
    struct notifier_block fb_notif;
    struct work_struct resume_work;
    pthread_mutex_t pm_lock;
    struct input_dev *input_dev;
    struct i2c_client *client;
    struct pdata *pdata;
};
static void *input_get_drvdata(struct input_dev *dev) { return dev->info; }
static int schedule_work(struct work_struct *w) {
    pthread_mutex_lock(&w->lock);
    int changed = !w->pending;
    w->pending = true;
    pthread_mutex_unlock(&w->lock);
    return changed;
}
static void cancel_work_sync(struct work_struct *w) {
    pthread_mutex_lock(&w->lock);
    w->pending = false;
    while (w->running) pthread_cond_wait(&w->done, &w->lock);
    pthread_mutex_unlock(&w->lock);
}
static int enabled, resumes, suspends, active, overlap;
static int entered, release_resume = 1, close_done, close_started;
static int load(int *p) { return __atomic_load_n(p, __ATOMIC_SEQ_CST); }
static void store(int *p, int v) { __atomic_store_n(p, v, __ATOMIC_SEQ_CST); }
static int fake_resume(struct device *dev) {
    if (__atomic_add_fetch(&active, 1, __ATOMIC_SEQ_CST) != 1) store(&overlap, 1);
    store(&entered, 1);
    while (!load(&release_resume)) usleep(100);
    usleep(1000);
    if (!enabled) { enabled = 1; resumes++; }
    __atomic_sub_fetch(&active, 1, __ATOMIC_SEQ_CST);
    return 0;
}
static int fake_suspend(struct device *dev) {
    if (__atomic_add_fetch(&active, 1, __ATOMIC_SEQ_CST) != 1) store(&overlap, 1);
    if (enabled) { enabled = 0; suspends++; }
    __atomic_sub_fetch(&active, 1, __ATOMIC_SEQ_CST);
    return 0;
}
#define RESUME fake_resume
#define SUSPEND fake_suspend
static int OPEN(struct input_dev *dev);
static void CLOSE(struct input_dev *dev);
'''

TEST = r'''
static void *run_work(void *arg) {
    struct INFO *info = arg;
    struct work_struct *w = &info->resume_work;
    pthread_mutex_lock(&w->lock);
    if (!w->pending) { pthread_mutex_unlock(&w->lock); return NULL; }
    w->pending = false; w->running = true;
    pthread_mutex_unlock(&w->lock);
    WORK(w);
    pthread_mutex_lock(&w->lock);
    w->running = false;
    pthread_cond_broadcast(&w->done);
    pthread_mutex_unlock(&w->lock);
    return NULL;
}
static void *close_input(void *arg) {
    struct INFO *info = arg;
    store(&close_started, 1);
    CLOSE(info->input_dev);
    store(&close_done, 1);
    return NULL;
}
static void *open_input(void *arg) {
    struct INFO *info = arg;
    OPEN(info->input_dev);
    return NULL;
}
static void notify(struct INFO *info, int blank) {
    struct fb_event event = { &blank };
    fb_notifier_callback(&info->fb_notif, FB_EVENT_BLANK, &event);
}
int main(void) {
    struct input_dev dev;
    struct i2c_client client;
    struct pdata pdata;
    struct INFO info = { .input_dev = &dev, .client = &client, .pdata = &pdata };
    dev.info = &info;
    pthread_mutex_init(&info.pm_lock, NULL);
    pthread_mutex_init(&info.resume_work.lock, NULL);
    pthread_cond_init(&info.resume_work.done, NULL);
    /* Unblank must return without doing any power/I2C operations. */
    notify(&info, FB_BLANK_UNBLANK);
    assert(!enabled && !resumes && info.resume_work.pending);
    /* A quick screen-off cancels a queued wake; no late resurrection. */
    notify(&info, FB_BLANK_POWERDOWN);
    run_work(&info);
    assert(!enabled && !resumes && !info.resume_work.pending);
    /* Duplicate wake requests coalesce. */
    notify(&info, FB_BLANK_UNBLANK);
    notify(&info, FB_BLANK_UNBLANK);
    run_work(&info);
    assert(enabled && resumes == 1);
    notify(&info, FB_BLANK_POWERDOWN);
    assert(!enabled && suspends == 1);
    /* Screen-off waits for in-flight power-on before powering off. */
    store(&entered, 0); store(&release_resume, 0);
    notify(&info, FB_BLANK_UNBLANK);
    pthread_t worker, closer, opener;
    pthread_create(&worker, NULL, run_work, &info);
    while (!load(&entered)) usleep(100);
    pthread_create(&closer, NULL, close_input, &info);
    while (!load(&close_started)) usleep(100);
    usleep(20000);
    assert(!load(&close_done));
    store(&release_resume, 1);
    pthread_join(worker, NULL); pthread_join(closer, NULL);
    assert(!enabled && resumes == 2 && suspends == 2 && !overlap);
    /* Input-core open and framebuffer work serialize controller access. */
    store(&entered, 0); store(&release_resume, 0);
    notify(&info, FB_BLANK_UNBLANK);
    pthread_create(&worker, NULL, run_work, &info);
    while (!load(&entered)) usleep(100);
    pthread_create(&opener, NULL, open_input, &info);
    usleep(20000);
    store(&release_resume, 1);
    pthread_join(worker, NULL); pthread_join(opener, NULL);
    assert(enabled && resumes == 3 && !overlap);
    notify(&info, FB_BLANK_POWERDOWN);
    assert(!enabled && suspends == 3);
    puts("PASS: deferred wake, cancellation, running wake/close, concurrent open");
}
'''

for path, prefix, info_type in [
    ("drivers/input/keyboard/abov_touchkey_a3.c", "abov_tk", "abov_tk_info"),
    ("drivers/input/touchscreen/zinitix/zinitix_bt541_ts.c", "bt541", "bt541_ts_info"),
]:
    source = (ROOT / path).read_text()
    probe_name = "abov_tk_probe" if prefix == "abov_tk" else "bt541_ts_probe"
    probe = source[source.index("static int " + probe_name + "("):
                   source.index("static int " + ("abov_tk_remove" if prefix == "abov_tk"
                                                 else "bt541_ts_remove") + "(")]
    assert source.count("INIT_WORK(&info->resume_work,") == 1
    assert probe.index("INIT_WORK(&info->resume_work,") < probe.index("input_register_device(")
    assert probe.index("mutex_init(&info->pm_lock)") < probe.index("input_register_device(")
    assert source.count("fb_register_client(&info->fb_notif)") == 1
    assert "fb_register_client(&info->fb_notif)" in probe
    names = {"INFO": info_type, "OPEN": prefix + "_input_open",
             "CLOSE": prefix + "_input_close", "WORK": prefix + "_resume_work",
             "RESUME": prefix + ("_resume" if prefix == "abov_tk" else "_ts_resume"),
             "SUSPEND": prefix + ("_suspend" if prefix == "abov_tk" else "_ts_suspend")}
    code = SHIM + "\n" + "\n".join([
        function(source, "static int " + names["OPEN"] + "(struct input_dev *dev)"),
        function(source, "static void " + names["CLOSE"] + "(struct input_dev *dev)"),
        function(source, "static void " + names["WORK"] + "(struct work_struct *work)"),
        function(source, "static int fb_notifier_callback(struct notifier_block *self,\n"
                 "\t\t\t\tunsigned long event, void *data)"),
    ]) + TEST
    # Replace shim placeholders only; driver function bodies stay unchanged.
    code = re.sub(r"\b(" + "|".join(names) + r")\b", lambda m: names[m[0]], code)
    with tempfile.TemporaryDirectory(prefix="a3-wake-test-") as temp:
        src = Path(temp) / "test.c"
        exe = Path(temp) / "test"
        src.write_text(code)
        subprocess.run([os.environ.get("HOSTCC", "cc"), "-pthread", "-O1", "-g",
                        "-fsanitize=address,undefined", str(src), "-o", str(exe)], check=True)
        subprocess.run([str(exe)], check=True, timeout=10)
    print(path)
