/******************************************************************************
 * ./arm_smmu.h
 *
 * Common compatibility defines and data_structures for porting arm smmu
 * drivers from Linux.
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

#ifndef __ARM_SMMU_H__
#define __ARM_SMMU_H__


/* Alias to Xen device tree helpers */
#define device_node dt_device_node
#define of_phandle_args dt_phandle_args
#define of_device_id dt_device_match
#define of_match_node dt_match_node
#define of_property_read_u32(np, pname, out) (!dt_property_read_u32(np, pname, out))
#define of_property_read_bool dt_property_read_bool
#define of_parse_phandle_with_args dt_parse_phandle_with_args

/* Helpers to get device MMIO and IRQs */
struct resource {
    u64 addr;
    u64 size;
    unsigned int type;
};

#define resource_size(res) ((res)->size)

#define platform_device device

#define IORESOURCE_MEM 0
#define IORESOURCE_IRQ 1

/* Stub out DMA domain related functions */
#define iommu_get_dma_cookie(dom) 0
#define iommu_put_dma_cookie(dom)

#define VA_BITS		0 /* Only used for configuring stage-1 input size */

#define MODULE_DEVICE_TABLE(type, name)
#define module_param_named(name, value, type, perm)
#define MODULE_PARM_DESC(_parm, desc)

#define dma_set_mask_and_coherent(d, b)	0
#define of_dma_is_coherent(n)	0

static void __iomem *devm_ioremap_resource(struct device *dev,
					   struct resource *res)
{
    void __iomem *ptr;

    if ( !res || res->type != IORESOURCE_MEM )
    {
        dev_err(dev, "Invalid resource\n");
        return ERR_PTR(-EINVAL);
    }

    ptr = ioremap_nocache(res->addr, res->size);
    if ( !ptr )
    {
        dev_err(dev, "ioremap failed (addr 0x%"PRIx64" size 0x%"PRIx64")\n",
                res->addr, res->size);
        return ERR_PTR(-ENOMEM);
    }

    return ptr;
}

/*
 * Domain type definitions. Not really needed for Xen, defining to port
 * Linux code as-is
 */
#define IOMMU_DOMAIN_UNMANAGED 0
#define IOMMU_DOMAIN_DMA 1
#define IOMMU_DOMAIN_IDENTITY 2

/* Xen: Compatibility define for iommu_domain_geometry.*/
struct iommu_domain_geometry {
    dma_addr_t aperture_start; /* First address that can be mapped    */
    dma_addr_t aperture_end;   /* Last address that can be mapped     */
    bool force_aperture;       /* DMA only allowed in mappable range? */
};

/* Xen: Dummy iommu_domain */
struct iommu_domain {
    /* Runtime SMMU configuration for this iommu_domain */
    struct arm_smmu_domain		*priv;
    unsigned int			type;

    /* Dummy compatibility defines */
    unsigned long pgsize_bitmap;
    struct iommu_domain_geometry geometry;

    atomic_t ref;
    /* Used to link iommu_domain contexts for a same domain.
     * There is at least one per-SMMU to used by the domain.
     */
    struct list_head		list;
};

/* Xen: Describes information required for a Xen domain */
struct arm_smmu_xen_domain {
    spinlock_t			lock;
    /* List of iommu domains associated to this domain */
    struct list_head		contexts;
};

#endif /* __ARM_SMMU_H__ */

