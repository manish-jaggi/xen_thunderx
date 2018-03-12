/*
 * xen/include/acpi/ridmap.h
 *
 * Manish Jaggi <manish.jaggi@linaro.org>
 * Copyright (c) 2018 Linaro.
 *
 * Ths program is free software; you can redistribute it and/or
 * modify it under the terms and conditions of the GNU General Public
 * License, version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ASM_ACPI_RIDMAP_H__
#define __ASM_ACPI_RIDMAP_H__

#include <xen/acpi.h>

/*
 * List holds requesterid (rid) - streamid (sid) mapping entries.
 */
extern struct list_head rid_sid_list;
/*
 * List holds requesterid (rid) - deviceid (devid) mapping entries.
 */
extern struct list_head rid_devid_list;

/*
 * structure to hold mapping between requresterid and streamid.
 * Note: output_reference and flags members of acpi_iort_id_mapping
 * are not used. This is done to avoid creating a new structure for
 * same purpose.
 *
 * smmu node pointer is stored in this structure because, in some places
 * smmu_node along with streamid is required based on rid and pcirc_node.
 */
struct rid_sid_map
{
    struct acpi_iort_node *pcirc_node;
    struct acpi_iort_node *smmu_node;
    struct acpi_iort_id_mapping idmap;
    struct list_head entry;
};

/*
 * API to add a rid-sid mapping
 * This method should be called while parsing each entry in idmap array
 * under the pcirc node in IORT.
 */
int add_rid_sid_map(struct acpi_iort_node *pcirc_node,
                    struct acpi_iort_node *smmu_node,
                    uint32_t input_base, uint32_t output_base,
                    uint32_t id_count);
/*
 * API to query sid and smmu_node based on pcirc_node and rid.
 *
 * Example of usage:
 *  int iort_pci_iommu_init(struct pci_dev *pdev, u16 alias, void *data)
 *  {
 *     struct iort_pci_alias_info *info = data;
 *   ...
 *      if ( query_sid(info->node, alias, &streamid, &smmu_node) )
 *          return iort_iommu_xlate(info->dev, smmu_node, streamid);
 *   ...
 *   }
 *
 */
bool query_sid(struct acpi_iort_node *pcirc_node, uint32_t rid,
               uint32_t *sid, struct acpi_iort_node **smmu_node);

/*
 * structure to hold a mapping between requresterid and deviceid.
 * Note: output_reference and flags members of acpi_iort_id_mapping
 * are not used. This is done to avoid creating a new structure for
 * same purpose.
 */
struct rid_devid_map
{
    struct acpi_iort_node *pcirc_node;
    struct acpi_iort_node *its_node;
    struct acpi_iort_id_mapping idmap;
    struct list_head entry;
};

/*
 * API to add a rid-devid mapping
 * This method should be called while parsing each entry in idmap array
 * under the pcirc node in IORT.
 */
int add_rid_devid_map(struct acpi_iort_node *pcirc_node,
                      struct acpi_iort_node *its_node,
                      uint32_t input_base, uint32_t output_base,
                      uint32_t id_count);

/*
 * API to query devid based on pcirc_node and rid */
bool query_devid(struct acpi_iort_node *pcirc_node, uint32_t rid,
                 uint32_t *devid);

#endif

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
