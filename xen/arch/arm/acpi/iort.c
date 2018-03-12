/*
 * Copyright (C) 2016, Semihalf
 *    Author: Tomasz Nowicki <tn@semihalf.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * This file implements early detection/parsing of I/O mapping
 * reported to OS through firmware via I/O Remapping Table (IORT)
 * IORT document number: ARM DEN 0049A
 *
 * Based on Linux 4.14.0
 * Xen Modifications : Manish Jaggi <manish.jaggi@linaro.org>
 * Coding Style: Xen
 */

#define pr_fmt(fmt)    "ACPI: IORT: " fmt

#include <asm/acpi/ridmap.h>
#include <asm/acpi/acpi_iort.h>
#include <asm/fwnode.h>
#include <asm/fwspec.h>
#include <xen/iommu.h>
#include <xen/kernel.h>
#include <xen/list.h>
#include <xen/lib.h>
#include <xen/pci.h>

#define IORT_TYPE_MASK(type)   (1 << (type))
#define IORT_MSI_TYPE          (1 << ACPI_IORT_NODE_ITS_GROUP)
#define IORT_IOMMU_TYPE        ((1 << ACPI_IORT_NODE_SMMU) |    \
                               (1 << ACPI_IORT_NODE_SMMU_V3))

/* Until ACPICA headers cover IORT rev. C */
#ifndef ACPI_IORT_SMMU_V3_CAVIUM_CN99XX
#define ACPI_IORT_SMMU_V3_CAVIUM_CN99XX        0x2
#endif

/* Redefine WARN macros */
#undef WARN
#undef WARN_ON
#define WARN(condition, format...) ({                    \
    int __ret_warn_on = !!(condition);                \
    if (unlikely(__ret_warn_on))                    \
        printk(format);                        \
    unlikely(__ret_warn_on);                    \
})
#define WARN_TAINT(cond, taint, format...) WARN(cond, format)
#define WARN_ON(cond)                      (!!cond)

#define MAX_ERRNO    4095
#define IS_ERR_VALUE(x) unlikely((unsigned long)(void *)(x) >= (unsigned long)-MAX_ERRNO)

struct iort_its_msi_chip {
    struct list_head list;
    struct fwnode_handle *fw_node;
    u32 translation_id;
};

struct iort_fwnode {
    struct list_head list;
    struct acpi_iort_node *iort_node;
    struct fwnode_handle *fwnode;
};

static LIST_HEAD(iort_fwnode_list);
static DEFINE_SPINLOCK(iort_fwnode_lock);
const struct fwnode_operations acpi_static_fwnode_ops;

/**
 * iort_set_fwnode() - Create iort_fwnode and use it to register
 *               iommu data in the iort_fwnode_list
 *
 * @node: IORT table node associated with the IOMMU
 * @fwnode: fwnode associated with the IORT node
 *
 * Returns: 0 on success
 *          <0 on failure
 */
static inline int iort_set_fwnode(struct acpi_iort_node *iort_node,
                  struct fwnode_handle *fwnode)
{
    struct iort_fwnode *np;

    np = xzalloc(struct iort_fwnode);

    if ( WARN_ON(!np) )
        return -ENOMEM;

    INIT_LIST_HEAD(&np->list);
    np->iort_node = iort_node;
    np->fwnode = fwnode;
    spin_lock(&iort_fwnode_lock);
    list_add_tail(&np->list, &iort_fwnode_list);
    spin_unlock(&iort_fwnode_lock);

    return 0;
}

/**
 * iort_get_fwnode() - Retrieve fwnode associated with an IORT node
 *
 * @node: IORT table node to be looked-up
 *
 * Returns: fwnode_handle pointer on success, NULL on failure
 */
static inline
struct fwnode_handle *iort_get_fwnode(struct acpi_iort_node *node)
{
    struct iort_fwnode *curr;
    struct fwnode_handle *fwnode = NULL;
    spin_lock(&iort_fwnode_lock);
    list_for_each_entry(curr, &iort_fwnode_list, list)
    {
        if ( curr->iort_node == node )
        {
            fwnode = curr->fwnode;
            break;
        }
    }
    spin_unlock(&iort_fwnode_lock);

    return fwnode;
}

/**
 * iort_delete_fwnode() - Delete fwnode associated with an IORT node
 *
 * @node: IORT table node associated with fwnode to delete
 */
static inline void iort_delete_fwnode(struct acpi_iort_node *node)
{
    struct iort_fwnode *curr, *tmp;

    spin_lock(&iort_fwnode_lock);
    list_for_each_entry_safe(curr, tmp, &iort_fwnode_list, list)
    {
        if ( curr->iort_node == node )
        {
            list_del(&curr->list);
            xfree(curr);
            break;
        }
    }
    spin_unlock(&iort_fwnode_lock);
}

typedef acpi_status (*iort_find_node_callback)
    (struct acpi_iort_node *node, void *context);

/* Root pointer to the mapped IORT table */
static struct acpi_table_header *iort_table;

static struct acpi_iort_node *iort_scan_node(enum acpi_iort_node_type type,
                         iort_find_node_callback callback,
                         void *context)
{
    struct acpi_iort_node *iort_node, *iort_end;
    struct acpi_table_iort *iort;
    int i;

    if ( !iort_table )
        return NULL;

    /* Get the first IORT node */
    iort = (struct acpi_table_iort *)iort_table;
    iort_node = ACPI_ADD_PTR(struct acpi_iort_node, iort,
                             iort->node_offset);
    iort_end = ACPI_ADD_PTR(struct acpi_iort_node, iort_table,
                            iort_table->length);

    for ( i = 0; i < iort->node_count; i++ )
    {
        if ( WARN_TAINT(iort_node >= iort_end, TAINT_FIRMWARE_WORKAROUND,
                   "IORT node pointer overflows, bad table!\n") )
            return NULL;

        if ( iort_node->type == type &&
             ACPI_SUCCESS(callback(iort_node, context)) )
            return iort_node;

        iort_node = ACPI_ADD_PTR(struct acpi_iort_node, iort_node,
                     iort_node->length);
    }

    return NULL;
}

static acpi_status iort_match_node_callback(struct acpi_iort_node *node,
                        void *context)
{
    struct device *dev = context;
    acpi_status status = AE_NOT_FOUND;

    if ( node->type == ACPI_IORT_NODE_PCI_ROOT_COMPLEX )
    {
        struct acpi_iort_root_complex *pci_rc;
        struct pci_dev *pdev;

        pdev = to_pci_dev(dev);
        pci_rc = (struct acpi_iort_root_complex *)node->node_data;

        /*
         * It is assumed that PCI segment numbers maps one-to-one
         * with root complexes. Each segment number can represent only
         * one root complex.
         */
        status = pci_rc->pci_segment_number == pci_domain_nr(pdev) ?
                                               AE_OK : AE_NOT_FOUND;
    }

    return status;
}

static int arm_smmu_iort_xlate(struct device *dev, u32 streamid,
                   struct fwnode_handle *fwnode,
                   const struct iommu_ops *ops)
{
    int ret;
    ret  = iommu_fwspec_init(dev, fwnode, ops);

    if ( !ret )
        ret = iommu_fwspec_add_ids(dev, &streamid, 1);

    return ret;
}

static inline
const struct iommu_ops *iort_fwspec_iommu_ops(struct iommu_fwspec *fwspec)
{
    return (fwspec && fwspec->ops) ? fwspec->ops : NULL;
}

static int iort_iommu_xlate(struct device *dev, struct acpi_iort_node *node,
                            u32 streamid)
{
    const struct iommu_ops *ops;
    struct fwnode_handle *iort_fwnode;

    if ( !node )
        return -ENODEV;

    iort_fwnode = iort_get_fwnode(node);
    if ( !iort_fwnode )
        return -ENODEV;

    /*
     * If the ops look-up fails, this means that either
     * the SMMU drivers have not been probed yet or that
     * the SMMU drivers are not built in the kernel;
     * Depending on whether the SMMU drivers are built-in
     * in the kernel or not, defer the IOMMU configuration
     * or just abort it.
     */
    ops = iommu_ops_from_fwnode(iort_fwnode);
    if (!ops)
        return -1;

    return arm_smmu_iort_xlate(dev, streamid, iort_fwnode, ops);
}

struct iort_pci_alias_info {
    struct device *dev;
    struct acpi_iort_node *node;
};

static int iort_pci_iommu_init(struct pci_dev *pdev, u16 alias, void *data)
{
    struct iort_pci_alias_info *info = data;
    struct acpi_iort_node *smmu_node;
    u32 streamid;

    if ( query_sid(info->node, alias, &streamid, &smmu_node) )
        return iort_iommu_xlate(info->dev, smmu_node, streamid);

    return -1; 
}

int pci_for_each_dma_alias(struct pci_dev *pdev,
               int (*fn)(struct pci_dev *pdev,
                   u16 alias, void *data), void *data)
{
    return fn(pdev, PCI_BDF2(pdev->bus, pdev->devfn), data);
}

/**
 * iort_iommu_configure - Set-up IOMMU configuration for a device.
 *
 * @dev: device to configure
 *
 * Returns: iommu_ops pointer on configuration success
 *          NULL on configuration failure
 */
const struct iommu_ops *iort_iommu_configure(struct device *dev)
{
    struct acpi_iort_node *node;
    const struct iommu_ops *ops;
    int err = -ENODEV;

    /*
     * If we already translated the fwspec there
     * is nothing left to do, return the iommu_ops.
     */
    ops = iort_fwspec_iommu_ops(dev->iommu_fwspec);
    if ( ops )
        return ops;

    if ( dev_is_pci(dev) )
    {
        struct iort_pci_alias_info info = { .dev = dev };
        node = iort_scan_node(
                              ACPI_IORT_NODE_PCI_ROOT_COMPLEX,
                              iort_match_node_callback,
                              dev);
        if ( !node )
            return NULL;

        info.node = node;
        err = pci_for_each_dma_alias(
                                     to_pci_dev(dev),
                                     iort_pci_iommu_init, &info);
    }

    return ops;
}

int iort_add_smmu_platform_device(struct acpi_iort_node *node)
{
    struct device *dev;
    struct fwnode_handle *fwnode;
    int ret;

    dev = xzalloc(struct device);
    if ( !dev )
        return -ENOMEM;

    dev->type = DEV_ACPI;
    dev->acpi_node = node;

    fwnode = iort_get_fwnode(node);

    if ( !fwnode )
    {
        ret = -ENODEV;
        goto end;
    }

    dev->fwnode = fwnode;
    dev->iommu_fwspec = xzalloc(struct iommu_fwspec);

    if ( !dev->iommu_fwspec )
    {    
        ret = -ENOMEM;
        goto end;
    }

    /* Call the acpi init functions for IOMMU devices */
    ret = acpi_device_init(DEVICE_IOMMU, (void *) dev, node->type);
end:
    return ret;
}

static inline struct fwnode_handle *acpi_alloc_fwnode_static(void)
{
   struct fwnode_handle *fwnode;

    fwnode = xzalloc(struct fwnode_handle);
    if ( !fwnode )
        return NULL;

    fwnode->ops = &acpi_static_fwnode_ops;

    return fwnode;
}

int __init acpi_iort_init(void)
{
    acpi_status status;

    status = acpi_get_table(ACPI_SIG_IORT, 0, &iort_table);

    if ( ACPI_FAILURE(status) )
    {
        if ( status != AE_NOT_FOUND )
        {
            const char *msg = acpi_format_exception(status);
            printk("Failed to get table, %s\n", msg);
        }

        return -1;
    }

    return 0;
}
__initcall(acpi_iort_init);
