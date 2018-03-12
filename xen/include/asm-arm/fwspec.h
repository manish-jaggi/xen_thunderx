/*
 *
 * Copyright (C) 2015, Intel Corporation
 * Author: Rafael J. Wysocki <rafael.j.wysocki@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Xen Modifications: Manish Jaggi
 */

#ifndef _ASM_FWSPEC_H
#define _ASM_FWSPEC_H

/**
 * struct iommu_fwspec - per-device IOMMU instance data
 * @ops: ops for this device's IOMMU
 * @iommu_fwnode: firmware handle for this device's IOMMU
 * @iommu_priv: IOMMU driver private data for this device
 * @num_ids: number of associated device IDs
 * @ids: IDs which this device may present to the IOMMU
 */
struct iommu_fwspec {
        const struct iommu_ops  *ops;
        struct fwnode_handle    *iommu_fwnode;
        void                    *iommu_priv;
        unsigned int            num_ids;
        u32                     ids[1];
};

int iommu_fwspec_init(struct device *dev, struct fwnode_handle *iommu_fwnode,
                      const struct iommu_ops *ops);
void iommu_fwspec_free(struct device *dev);
int iommu_fwspec_add_ids(struct device *dev, u32 *ids, int num_ids);
const struct iommu_ops *iommu_ops_from_fwnode(struct fwnode_handle *fwnode);

#endif
