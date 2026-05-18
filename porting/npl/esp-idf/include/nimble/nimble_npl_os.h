/*
 * SPDX-FileCopyrightText: 2015-2022 The Apache Software Foundation (ASF)
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * SPDX-FileContributor: 2025 Espressif Systems (Shanghai) CO LTD
 */

#ifndef _NIMBLE_NPL_OS_H_
#define _NIMBLE_NPL_OS_H_

#include <stdint.h>
#include "btdm_osal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(array) \
        (sizeof(array) / sizeof((array)[0]))
#endif

#define BLE_NPL_OS_ALIGNMENT (4) /*ble_npl_get_os_alignment()*/

#define BLE_NPL_TIME_FOREVER portMAX_DELAY

/* This should be compatible with TickType_t */
typedef uint32_t ble_npl_time_t;
typedef int32_t ble_npl_stime_t;

struct ble_npl_event {
    struct btdm_osal_event event;
};

struct ble_npl_eventq {
    struct btdm_osal_eventq eventq;
};

struct ble_npl_callout {
    struct btdm_osal_callout co;
};

struct ble_npl_mutex {
    struct btdm_osal_mutex mutex;
};

struct ble_npl_sem {
    struct btdm_osal_sem sem;
};

/*
 * Simple APIs are just defined as static inline below, but some are a bit more
 * complex or require some global state variables and thus are defined in .c
 * file instead and static inline wrapper just calls proper implementation.
 * We need declarations of these functions and they are defined in header below.
 */
static inline bool
ble_npl_os_started(void)
{
    return xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED;
}

static inline void *
ble_npl_get_current_task_id(void)
{
    return xTaskGetCurrentTaskHandle();
}

static inline void
ble_npl_eventq_init(struct ble_npl_eventq *evq)
{
    return btdm_osal_eventq_init(&evq->eventq);
}

static inline void
ble_npl_eventq_deinit(struct ble_npl_eventq *evq)
{
    return btdm_osal_eventq_deinit(&evq->eventq);
}

static inline struct ble_npl_event *
ble_npl_eventq_get(struct ble_npl_eventq *evq, ble_npl_time_t tmo)
{
    struct btdm_osal_event *ev;

    ev = btdm_osal_eventq_get(&evq->eventq, tmo);
    if (ev) {
        return (struct ble_npl_event *)((char *)ev - offsetof(struct ble_npl_event, event));
    }
    return NULL;
}

static inline void
ble_npl_eventq_put(struct ble_npl_eventq *evq, struct ble_npl_event *ev)
{
    btdm_osal_eventq_put(&evq->eventq, &ev->event);
}

static inline void
ble_npl_eventq_remove(struct ble_npl_eventq *evq, struct ble_npl_event *ev)
{
    btdm_osal_eventq_remove(&evq->eventq, &ev->event);
}

static inline void
ble_npl_event_run(struct ble_npl_event *ev)
{
    btdm_osal_event_run((struct btdm_osal_event *)ev);
}

static inline bool
ble_npl_eventq_is_empty(struct ble_npl_eventq *evq)
{
    return btdm_osal_eventq_is_empty(&evq->eventq);
}

static inline void
ble_npl_event_init(struct ble_npl_event *ev, ble_npl_event_fn *fn, void *arg)
{
    btdm_osal_event_init(&ev->event, (btdm_osal_event_fn *)fn, arg);
}
static inline void
ble_npl_event_deinit(struct ble_npl_event *ev)
{
    btdm_osal_event_deinit(&ev->event);
}

static inline void
ble_npl_event_reset(struct ble_npl_event *ev)
{
    btdm_osal_event_reset(&ev->event);
}

static inline bool
ble_npl_event_is_queued(struct ble_npl_event *ev)
{
    return btdm_osal_event_is_queued(&ev->event);
}

static inline void *
ble_npl_event_get_arg(struct ble_npl_event *ev)
{
    return btdm_osal_event_get_arg(&ev->event);
}

static inline void
ble_npl_event_set_arg(struct ble_npl_event *ev, void *arg)
{
    return btdm_osal_event_set_arg(&ev->event, arg);
}

static inline ble_npl_error_t
ble_npl_mutex_init(struct ble_npl_mutex *mu)
{
    return (ble_npl_error_t)btdm_osal_mutex_init(&mu->mutex);
}

static inline ble_npl_error_t
ble_npl_mutex_deinit(struct ble_npl_mutex *mu)
{
    return (ble_npl_error_t)btdm_osal_mutex_deinit(&mu->mutex);
}

static inline ble_npl_error_t
ble_npl_mutex_pend(struct ble_npl_mutex *mu, ble_npl_time_t timeout)
{
    return (ble_npl_error_t)btdm_osal_mutex_pend(&mu->mutex, timeout);
}

static inline ble_npl_error_t
ble_npl_mutex_release(struct ble_npl_mutex *mu)
{
    return (ble_npl_error_t)btdm_osal_mutex_release(&mu->mutex);
}

static inline ble_npl_error_t
ble_npl_sem_init(struct ble_npl_sem *sem, uint16_t tokens)
{
    return (ble_npl_error_t)btdm_osal_sem_init(&sem->sem, tokens);
}

static inline ble_npl_error_t
ble_npl_sem_deinit(struct ble_npl_sem *sem)
{
    return (ble_npl_error_t)btdm_osal_sem_deinit(&sem->sem);
}

static inline ble_npl_error_t
ble_npl_sem_pend(struct ble_npl_sem *sem, ble_npl_time_t timeout)
{
    return (ble_npl_error_t)btdm_osal_sem_pend(&sem->sem, timeout);
}

static inline ble_npl_error_t
ble_npl_sem_release(struct ble_npl_sem *sem)
{
    return (ble_npl_error_t)btdm_osal_sem_release(&sem->sem);
}

static inline uint16_t
ble_npl_sem_get_count(struct ble_npl_sem *sem)
{
    return btdm_osal_sem_get_count(&sem->sem);
}

static inline int
ble_npl_callout_init(struct ble_npl_callout *co, struct ble_npl_eventq *evq,
                     ble_npl_event_fn *ev_cb, void *ev_arg)
{
    return btdm_osal_callout_init(&co->co, &evq->eventq, (btdm_osal_event_fn *)ev_cb, ev_arg);
}

static inline void
ble_npl_callout_deinit(struct ble_npl_callout *co)
{
    return btdm_osal_callout_deinit(&co->co);
}

static inline ble_npl_error_t
ble_npl_callout_reset(struct ble_npl_callout *co, ble_npl_time_t ticks)
{
    return (ble_npl_error_t)btdm_osal_callout_reset(&co->co, ticks);
}

static inline void
ble_npl_callout_stop(struct ble_npl_callout *co)
{
    return btdm_osal_callout_stop(&co->co);
}

static inline bool
ble_npl_callout_is_active(struct ble_npl_callout *co)
{
    return btdm_osal_callout_is_active(&co->co);
}

static inline ble_npl_time_t
ble_npl_callout_get_ticks(struct ble_npl_callout *co)
{
    return btdm_osal_callout_get_ticks(&co->co);
}

static inline ble_npl_time_t
ble_npl_callout_remaining_ticks(struct ble_npl_callout *co, ble_npl_time_t time)
{
    return btdm_osal_callout_remaining_ticks(&co->co, time);
}

static inline void
ble_npl_callout_set_arg(struct ble_npl_callout *co, void *arg)
{
    return btdm_osal_callout_set_arg(&co->co, arg);
}

static inline ble_npl_time_t
ble_npl_time_get(void)
{
    return btdm_osal_time_get();
}

static inline ble_npl_error_t
ble_npl_time_ms_to_ticks(uint32_t ms, ble_npl_time_t *out_ticks)
{
    return (ble_npl_error_t)btdm_osal_time_ms_to_ticks(ms, out_ticks);
}

static inline ble_npl_error_t
ble_npl_time_ticks_to_ms(ble_npl_time_t ticks, uint32_t *out_ms)
{
    return (ble_npl_error_t)btdm_osal_time_ticks_to_ms(ticks, out_ms);
}

static inline ble_npl_time_t
ble_npl_time_ms_to_ticks32(uint32_t ms)
{
    return btdm_osal_time_ms_to_ticks32(ms);
}

static inline uint32_t
ble_npl_time_ticks_to_ms32(ble_npl_time_t ticks)
{
    return btdm_osal_time_ticks_to_ms32(ticks);
}

static inline void
ble_npl_time_delay(ble_npl_time_t ticks)
{
    vTaskDelay(ticks);
}

// #if NIMBLE_CFG_CONTROLLER
// static inline void
// ble_npl_hw_set_isr(int irqn, uint32_t addr)
// {
//     return btdm_osal_hw_set_isr(irqn, addr);
// }
// #endif

static inline uint32_t
ble_npl_hw_enter_critical(void)
{
    return btdm_osal_hw_enter_critical();
}

static inline void
ble_npl_hw_exit_critical(uint32_t ctx)
{
    btdm_osal_hw_exit_critical(ctx);
}

static inline bool
ble_npl_hw_is_in_critical(void)
{
    return btdm_osal_hw_is_in_critical();
}

#ifdef __cplusplus
}
#endif

#endif /* _NPL_H_ */
