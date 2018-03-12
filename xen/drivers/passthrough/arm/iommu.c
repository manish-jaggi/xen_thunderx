/*
 * Generic IOMMU framework via the device tree
 *
 * Julien Grall <julien.grall@linaro.org>
 * Copyright (c) 2014 Linaro Limited.
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
 */

#include <xen/lib.h>
#include <xen/iommu.h>
#include <xen/device_tree.h>
#include <asm/device.h>
#include <asm/fwnode.h>
#include <asm/fwspec.h>

static const struct iommu_ops *iommu_ops;

const struct iommu_ops *iommu_get_ops(void)
{
    return iommu_ops;
}

void __init iommu_set_ops(const struct iommu_ops *ops)
{
    BUG_ON(ops == NULL);

    if ( iommu_ops && iommu_ops != ops )
        printk("WARNING: Cannot set IOMMU ops, already set to a different value\n");

    iommu_ops = ops;
}

int __init iommu_hardware_setup(void)
{
    struct dt_device_node *np;
    int rc;
    unsigned int num_iommus = 0;

    dt_for_each_device_node(dt_host, np)
    {
        rc = device_init(np, DEVICE_IOMMU, NULL);
        if ( !rc )
            num_iommus++;
    }

    return ( num_iommus > 0 ) ? 0 : -ENODEV;
}

void __hwdom_init arch_iommu_check_autotranslated_hwdom(struct domain *d)
{
    /* ARM doesn't require specific check for hwdom */
    return;
}

int arch_iommu_domain_init(struct domain *d)
{
    return iommu_dt_domain_init(d);
}

void arch_iommu_domain_destroy(struct domain *d)
{
}

int arch_iommu_populate_page_table(struct domain *d)
{
    /* The IOMMU shares the p2m with the CPU */
    return -ENOSYS;
}

/**
 * fwnode_handle_put - Drop reference to a device node
 * @fwnode: Pointer to the device node to drop the reference to.
 *
 * To be used when terminating device_for_each_child_node() iteration with 
 * break / return to prevent stale device node references being left behind
 */
void fwnode_handle_put(struct fwnode_handle *fwnode)
{
    fwnode_call_void_op(fwnode, put);
}

const struct iommu_ops *iommu_ops_from_fwnode(struct fwnode_handle *fwnode)
{
    return iommu_get_ops();
}

int iommu_fwspec_init(struct device *dev, struct fwnode_handle *iommu_fwnode,
                      const struct iommu_ops *ops)
{
    struct iommu_fwspec *fwspec = dev->iommu_fwspec;

    if ( fwspec )
       return ops == fwspec->ops ? 0 : -EINVAL;

    fwspec = xzalloc_bytes(sizeof(*fwspec));
    if ( !fwspec )
        return -ENOMEM;
#if 0 
       of_node_get(to_of_node(iommu_fwnode)); /* TODO */
#endif
    fwspec->iommu_fwnode = iommu_fwnode;
    fwspec->ops = ops;
    dev->iommu_fwspec = fwspec;

    return 0;
}

void iommu_fwspec_free(struct device *dev)
{
   struct iommu_fwspec *fwspec = dev->iommu_fwspec;

    if ( fwspec )
    {
        fwnode_handle_put(fwspec->iommu_fwnode);
        xfree(fwspec);
        dev->iommu_fwspec = NULL;
    }
}

int iommu_fwspec_add_ids(struct device *dev, u32 *ids, int num_ids)
{
    struct iommu_fwspec *fwspec_n;
    struct iommu_fwspec *fwspec = dev->iommu_fwspec;
    size_t size, size_n;
    int i;

    if ( !fwspec )
        return -EINVAL;

    size = offsetof(struct iommu_fwspec, ids[fwspec->num_ids]);
    size_n = offsetof(struct iommu_fwspec, ids[fwspec->num_ids + num_ids]);
    if ( size_n > size )
    {
        fwspec_n = _xzalloc(size_n, sizeof(void*));
        if ( !fwspec_n )
             return -ENOMEM;
        
        memcpy(fwspec_n, fwspec, size);
        xfree(fwspec);
        fwspec = fwspec_n;
    }

    for ( i = 0; i < num_ids; i++ )
        fwspec->ids[fwspec->num_ids + i] = ids[i];

    fwspec->num_ids += num_ids;
    dev->iommu_fwspec = fwspec;

    return 0;
}

