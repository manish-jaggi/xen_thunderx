/******************************************************************************
 * include/xen/linux_compat.h
 *
 * Compatibility defines for porting code from Linux to Xen
 *
 * Copyright (c) 2017 Linaro Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __XEN_LINUX_COMPAT_H__
#define __XEN_LINUX_COMPAT_H__

#include <asm/types.h>

typedef paddr_t phys_addr_t;
typedef paddr_t dma_addr_t;

typedef unsigned int gfp_t;
#define GFP_KERNEL 0
#define __GFP_ZERO 0x01U

/* Helpers for IRQ functions */
#define free_irq release_irq

enum irqreturn {
    IRQ_NONE,
    IRQ_HANDLED,
    IRQ_WAKE_THREAD,
};

typedef enum irqreturn irqreturn_t;

/* Device logger functions */
#define dev_dbg(dev, fmt, ...) printk(XENLOG_DEBUG fmt, ## __VA_ARGS__)
#define dev_notice(dev, fmt, ...) printk(XENLOG_INFO fmt, ## __VA_ARGS__)
#define dev_warn(dev, fmt, ...) printk(XENLOG_WARNING fmt, ## __VA_ARGS__)
#define dev_err(dev, fmt, ...) printk(XENLOG_ERR fmt, ## __VA_ARGS__)
#define dev_info(dev, fmt, ...) printk(XENLOG_INFO fmt, ## __VA_ARGS__)

#define dev_err_ratelimited(dev, fmt, ...)                  \
     printk(XENLOG_ERR fmt, ## __VA_ARGS__)

#define dev_name(dev) dt_node_full_name(dev_to_dt(dev))

/* Alias to Xen allocation helpers */
#define kfree xfree
#define kmalloc(size, flags) ({\
	void *__ret_alloc = NULL; \
	if (flags & __GFP_ZERO) \
		__ret_alloc = _xzalloc(size, sizeof(void *)); \
	else \
		__ret_alloc = _xmalloc(size, sizeof(void *)); \
	__ret_alloc; \
})
#define kzalloc(size, flags)        _xzalloc(size, sizeof(void *))
#define devm_kzalloc(dev, size, flags)  _xzalloc(size, sizeof(void *))
#define kmalloc_array(size, n, flags) ({\
	void *__ret_alloc = NULL; \
	if (flags & __GFP_ZERO) \
		__ret_alloc = _xzalloc_array(size, sizeof(void *), n); \
	else \
		__ret_alloc = _xmalloc_array(size, sizeof(void *), n); \
	__ret_alloc; \
})

/* Alias to Xen time functions */
#define ktime_t s_time_t
#define ktime_get()             (NOW())
#define ktime_add_us(t,i)       (t + MICROSECS(i))
#define ktime_compare(t,i)      (t > (i))

#endif /* __XEN_LINUX_COMPAT_H__ */
