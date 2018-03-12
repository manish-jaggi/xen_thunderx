/*
 * xen/arch/arm/acpi/gen-iort.c
 *
 * Code to generate IORT for hardware domain using the requesterId
 * and deviceId map.
 *
 * Manish Jaggi <manish.jaggi@linaro.com>
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
#include <xen/acpi.h>

/*
 * Size of hardware domains' IORT is calculated based on the number of
 * mappings in the requesterid - deviceid mapping list.
 * Return value 0: Success
 */
int estimate_iort_size(size_t *iort_size)
{
    int count = 0;
    int pcirc_count = 0;
    int itsg_count = 0;
    uint64_t *pcirc_array;
    uint64_t *itsg_array;
    struct rid_devid_map *rmap;

    list_for_each_entry(rmap, &rid_devid_list, entry)
        count++;

    pcirc_array = xzalloc_bytes(sizeof(uint64_t)*count);
    if ( !pcirc_array )
        return -ENOMEM;

    itsg_array = xzalloc_bytes(sizeof(uint64_t)*count);
    if ( !itsg_array )
        return -ENOMEM;

    list_for_each_entry(rmap, &rid_devid_list, entry)
    {
        int i = 0;

        for ( i = 0; i <= pcirc_count; i++ )
        {
            if ( pcirc_array[i] == (uint64_t) rmap->pcirc_node )
                break;
            if ( i == pcirc_count )
            {
                pcirc_array[i] = (uint64_t) rmap->pcirc_node;
                pcirc_count++;
                break;
            }
        }

        for ( i = 0; i <= itsg_count; i++ )
        {
            if ( itsg_array[i] == (uint64_t) rmap->its_node )
                break;
            if ( i == itsg_count )
            {
                itsg_array[i] = (uint64_t) rmap->its_node;
                itsg_count++;
                break;
            }
        }
    }

    /* Size of IORT
     * = Size of IORT Table Header + Size of PCIRC Header Nodes +
     *   Size of PCIRC nodes + Size of ITS Header nodes + Size of ITS Nodes
     *   + Size of idmap nodes
     */
    *iort_size = sizeof(struct acpi_table_iort) +
                 pcirc_count*( (sizeof(struct acpi_iort_node) -1) +
                               sizeof(struct acpi_iort_root_complex) ) +
                 itsg_count*( (sizeof(struct acpi_iort_node) -1) +
                               sizeof(struct acpi_iort_its_group) ) +
                 count*( sizeof(struct acpi_iort_id_mapping) );

    xfree(itsg_array);
    xfree(pcirc_array);

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
