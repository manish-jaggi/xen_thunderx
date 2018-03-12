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
#include <acpi/actables.h>

/*
 * Structure of Hardware domain's (hwdom) IORT
 * -----------------------------------
 *
 * hwdom's IORT will only have PCIRC nodes and ITS group nodes
 * in the following order.
 *
 * [IORT Header]
 * [ITS Group 1 ]
 * ...
 * [ITS Group N ]
 * [PCIRC Node 1]
 * [PCIRC IDMAP entry 1]
 * ...
 * [PCIRC IDMAP entry N]
 * ...
 * [PCIRC Node N]
 *
 * requesterid-deviceid mapping list (rid_devid_list) populated by parsing IORT
 * is used to generate hwdom IORT.
 *
 * One of the challanges is to fixup node offset of ITS Group Nodes
 * in the PCIRC idmap (output_reference)
 *
 * In rid_devid_map firmware IORT's ITS group node pointer in stored.
 *
 * We first write all the ITS group nodes in the hwdom's IORT. For this
 * write_hwits_nodes is called, which parses the rid_devid_list and for
 * each unique its_node in firmware IORT create a its_node in hwdom's IORT
 * and also creates and entry in fwits_hwits_map.
 *
 * fwits_hwits_map is a mapping between firmware IORT's its node
 * and the node offset of the corresponding its_node stored in the
 * hwdom's IORT.
 *
 * This map can be later used to set output reference value in hwdom's
 * pcirc node's idmap entries.
 *
 */

/*
 * Stores the mapping between firmware tables its group node
 * to the offset of the equivalent its node to be stored in
 * hwdom's IORT.
 */
struct fwits_hwits_map
{
    struct acpi_iort_node *fwits_node;
    unsigned int hwitsnode_offset;
    struct list_head entry;
};

LIST_HEAD(fwits_hwits_list);

/*
 * is_uniq_fwits_node
 *
 * returns 1 - if fwits_node is not already in the its_map_list
 *         0 - if it is present already
 *
 * fwits_node - ITS Node pointer in Firmware IORT
 * offset     - offset of the equivalent its node to be stored in
 *              hwdom's IORT
 */
static int is_uniq_fwits_node(struct acpi_iort_node *fwits_node,
                              unsigned int offset)
{
    struct fwits_hwits_map *map;

    list_for_each_entry(map, &fwits_hwits_list, entry)
    {
        if ( map->fwits_node == fwits_node )
            return 0;
    }

    map = xzalloc(struct fwits_hwits_map);
    if ( !map )
        return -ENOMEM;

    map->fwits_node = fwits_node;
    map->hwitsnode_offset = offset;
    list_add_tail(&map->entry, &fwits_hwits_list);

    return 1;
}

/*
 * Returns the offset of corresponding its node to fwits_node
 * written in hwdom's IORT.
 *
 * This function would be used when write hwdoms pcirc nodes' idmap
 * entries.
 */
static
unsigned int hwitsnode_offset_from_map(struct acpi_iort_node *fwits_node)
{
    struct fwits_hwits_map *map;

    list_for_each_entry(map, &fwits_hwits_list, entry)
    {
        if ( map->fwits_node == fwits_node )
            return map->hwitsnode_offset;
    }

    return 0;
}

static void write_hwits_nodes(u8 *iort, unsigned int *offset,
                              unsigned int *num_nodes)
{
    struct rid_devid_map *rmap;
    unsigned int of = *offset;
    int n = 0;

    /*
     * rid_devid_list is iterated to get unique its group nodes
     * Each unique ITS group node is written in hardware domains IORT
     * by using some values from the firmware ITS group node.
     */
    list_for_each_entry(rmap, &rid_devid_list, entry)
    {
        struct acpi_iort_node *node;
        struct acpi_iort_its_group *grp;
        struct acpi_iort_its_group *fw_grp;

        /* save its_node_offset_map in a list uniquely */
        if ( is_uniq_fwits_node(rmap->its_node, of) == 1 )
        {
            node = (struct acpi_iort_node *) &iort[of];
            grp = (struct acpi_iort_its_group *)(&node->node_data);

            node->type = ACPI_IORT_NODE_ITS_GROUP;
            node->length = sizeof(struct acpi_iort_node) +
                           sizeof(struct acpi_iort_its_group) -
                           sizeof(node->node_data);

            node->revision = rmap->its_node->revision;
            node->reserved = 0;
            node->mapping_count = 0;
            node->mapping_offset= 0;

            fw_grp = (struct acpi_iort_its_group *)(&rmap->its_node->node_data);

            /* Copy its_count and identifiers from firmware iort's its_node */
            grp->its_count = fw_grp->its_count;
            grp->identifiers[0] = fw_grp->identifiers[0];

            of += node->length;
            n++;
        }
    }
    *offset = of;
    *num_nodes = n;
}

static void write_hwpcirc_nodes(u8 *iort, unsigned int *pos,
                                unsigned int *num_nodes)
{
    struct acpi_iort_node *opcirc_node, *pcirc_node;
    struct acpi_iort_node *hwdom_pcirc_node = NULL;
    struct rid_devid_map *rmap;
    struct acpi_iort_id_mapping *idmap;
    int num_idmap = 0, n = 0;
    unsigned int old_pos = *pos;

    opcirc_node = NULL;
    /* Iterate rid_map_devid list */
    list_for_each_entry(rmap, &rid_devid_list, entry)
    {
        struct acpi_iort_root_complex *rc;
        struct acpi_iort_root_complex *rc_fw;
        int add_node = 0;

        pcirc_node = rmap->pcirc_node;

        if ( opcirc_node == NULL ) /* First entry */
        {
            add_node = 1;
        }
        else if ( opcirc_node != pcirc_node ) /* another pci_rc_node found*/
        {
            /* All the idmaps of a pcirc are written, now update node info*/
            hwdom_pcirc_node->length = num_idmap *
                                       sizeof(struct acpi_iort_id_mapping) +
                                       sizeof(struct acpi_iort_node) +
                                       sizeof(struct acpi_iort_root_complex) -
                                       sizeof(pcirc_node->node_data);

            hwdom_pcirc_node->mapping_count = num_idmap;
            hwdom_pcirc_node->mapping_offset = sizeof(struct acpi_iort_node) +
                                               sizeof(struct acpi_iort_root_complex) -
                                               sizeof(pcirc_node->node_data);
            old_pos += hwdom_pcirc_node->length;
            add_node = 1;
        }

        if ( add_node ) /* create the pcirc node */
        {
            opcirc_node = pcirc_node;
            hwdom_pcirc_node = (struct acpi_iort_node *)&iort[old_pos];
            hwdom_pcirc_node->type = ACPI_IORT_NODE_PCI_ROOT_COMPLEX;
            hwdom_pcirc_node->mapping_offset = sizeof(struct acpi_iort_node) +
                                               sizeof(struct acpi_iort_root_complex) -
                                               sizeof(hwdom_pcirc_node->node_data);

            rc = (struct acpi_iort_root_complex *)
                  &hwdom_pcirc_node->node_data;

            rc_fw = (struct acpi_iort_root_complex *)
                     &pcirc_node->node_data;

            rc->pci_segment_number = rc_fw->pci_segment_number;
            rc->ats_attribute = rc_fw->ats_attribute;
            rc->memory_properties = rc_fw->memory_properties;

            idmap = ACPI_ADD_PTR(struct acpi_iort_id_mapping,
                                 hwdom_pcirc_node,
                                 hwdom_pcirc_node->mapping_offset);
            n++;
            num_idmap = 0;
        }

        idmap->input_base = rmap->idmap.input_base;
        idmap->id_count = rmap->idmap.id_count;
        idmap->output_base = rmap->idmap.output_base;
        idmap->output_reference = hwitsnode_offset_from_map(rmap->its_node);
        BUG_ON(!idmap->output_reference);

        idmap->flags = 0;
        idmap++;
        num_idmap++;
    }

    if ( hwdom_pcirc_node ) /* if no further PCIRC nodes found */
    {
        /* All the idmaps of a pcirc are written, now update node info*/
        hwdom_pcirc_node->length = num_idmap *
                                   sizeof(struct acpi_iort_id_mapping) +
                                   sizeof(struct acpi_iort_node) +
                                   sizeof(struct acpi_iort_root_complex) -1;

        hwdom_pcirc_node->mapping_count = num_idmap;
        old_pos += hwdom_pcirc_node->length;
    }

    *pos = old_pos;
    *num_nodes = n;
}

bool is_iort_available(void)
{
    struct acpi_table_iort *iort;

    if ( acpi_get_table(ACPI_SIG_IORT, 0,
         (struct acpi_table_header **)&iort) )
        return 0;

    return 1;
}

int prepare_hwdom_iort(struct acpi_table_iort *hwdom_iort, unsigned int *iort_size)
{
    struct acpi_table_iort *fw_iort;
    unsigned int num_nodes = 0;
    unsigned int pos;

    pos = sizeof(struct acpi_table_iort);

    if ( acpi_get_table(ACPI_SIG_IORT, 0,
         (struct acpi_table_header **)&fw_iort) )
    {
        printk("Failed to get IORT table\n");
        return -ENODEV;
    }

    /* Write IORT header */
    ACPI_MEMCPY(hwdom_iort, fw_iort, sizeof(struct acpi_table_iort));
    hwdom_iort->node_offset = pos;
    hwdom_iort->node_count = 0;

    /* Write its group nodes */
    write_hwits_nodes((u8*)hwdom_iort, &pos, &num_nodes);
    hwdom_iort->node_count = num_nodes;

    /* Write pcirc_nodes */
    write_hwpcirc_nodes((u8*)hwdom_iort, &pos, &num_nodes);

    /* Update IORT Size in IORT header */
    hwdom_iort->node_count += num_nodes;
    hwdom_iort->header.length = pos;
    hwdom_iort->header.checksum =  acpi_tb_checksum(
                                    ACPI_CAST_PTR(u8, hwdom_iort),
                                    pos);
    *iort_size = hwdom_iort->header.length;

    return 0;
}

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
