/*
 * xen/include/asm-arm/acpi/gen-iort.h
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

#ifndef _ACPI_GEN_IORT_H
#define _ACPI_GEN_IORT_H

/*
 * Returns the size of hardware domains IORT
 */ 
int estimate_iort_size(size_t *iort_size);

/*
 * Checks if IORT is present in ACPI tables.
 */
bool is_iort_available(void);

/*
 * Prepares IORT in buffer hwdom_iort and updates iort_size
 */
int prepare_hwdom_iort(struct acpi_table_iort *hwdom_iort,
                       unsigned int *iort_size);
 
#endif
/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
