/*
 * xen/drivers/acpi/arm/ridmap.c
 *
 * This file implements rid-sid rid-devid mapping API
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

#include <asm/acpi/ridmap.h>
#include <xen/iommu.h>
#include <xen/kernel.h>
#include <xen/list.h>
#include <xen/pci.h>

LIST_HEAD(rid_sid_list);
LIST_HEAD(rid_devid_list);

int add_rid_sid_map(struct acpi_iort_node *pcirc_node,
                    struct acpi_iort_node *smmu_node,
                    uint32_t input_base, uint32_t output_base,
                    uint32_t id_count)
{
    struct rid_sid_map *rid_map;

    rid_map = xzalloc(struct rid_sid_map);
    if ( !rid_map )
        return -ENOMEM;

    rid_map->idmap.input_base = input_base;
    rid_map->idmap.output_base = output_base;
    rid_map->idmap.id_count = id_count;
    rid_map->pcirc_node = pcirc_node;
    rid_map->smmu_node = smmu_node;

    list_add_tail(&rid_map->entry, &rid_sid_list);

    return 0;
}

int add_rid_devid_map(struct acpi_iort_node *pcirc_node,
                      struct acpi_iort_node *its_node,
                      uint32_t input_base, uint32_t output_base,
                      uint32_t id_count)
{
    struct rid_devid_map *rid_map;

    rid_map = xzalloc(struct rid_devid_map);
    if ( !rid_map )
        return -ENOMEM;

    rid_map->idmap.input_base = input_base;
    rid_map->idmap.output_base = output_base;
    rid_map->idmap.id_count = id_count;
    rid_map->pcirc_node = pcirc_node;
    rid_map->its_node = its_node;

    list_add_tail(&rid_map->entry, &rid_devid_list);

    return 0;
}

bool query_sid(struct acpi_iort_node *pcirc_node, uint32_t rid,
               uint32_t *sid, struct acpi_iort_node **smmu_node)
{
    struct rid_sid_map *rmap;

    list_for_each_entry(rmap, &rid_sid_list, entry)
    {
        if ( rmap->pcirc_node == pcirc_node )
        {
            if ( (rid >= rmap->idmap.input_base) &&
                 (rid < rmap->idmap.input_base + rmap->idmap.id_count) )
            {
                *sid = rid - rmap->idmap.input_base +
                       rmap->idmap.output_base;
                *smmu_node = rmap->smmu_node;

                return 1;
            }
        }
    }

    return 0;
}

bool query_devid(struct acpi_iort_node *pcirc_node,
                uint32_t rid, uint32_t *devid)
{
    struct rid_devid_map *rmap;

    list_for_each_entry(rmap, &rid_devid_list, entry)
    {
        if ( rmap->pcirc_node == pcirc_node )
        {
            if ( (rid >= rmap->idmap.input_base) &&
                 (rid < rmap->idmap.input_base + rmap->idmap.id_count) )
            {
                *devid = rid - rmap->idmap.input_base +
                         rmap->idmap.output_base;

                return 1;
            }
        }
    }

    return 0;
}

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
