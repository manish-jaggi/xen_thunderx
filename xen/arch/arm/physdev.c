/******************************************************************************
 * Arch-specific physdev.c
 *
 * Copyright (c) 2012, Citrix Systems
 */

#include <xen/types.h>
#include <xen/lib.h>
#include <xen/errno.h>
#include <xen/sched.h>
#include <asm/hypercall.h>
#include <xen/guest_access.h>
 
 
extern int acpi_setup_smmu_for_test(struct domain *d, u16 seg, u8 bus, u8 devfn);

 int do_physdev_op(int cmd, XEN_GUEST_HANDLE_PARAM(void) arg)
 {
     int ret = 0;
 
     switch (cmd )
     {
     case PHYSDEVOP_pci_device_add: {
         struct physdev_pci_device_add add;
         struct pci_dev_info pdev_info;
 
         ret = -EFAULT;
         if ( copy_from_guest(&add, arg, 1) != 0 )
             break;
 
         pdev_info.is_extfn = !!(add.flags & XEN_PCI_DEV_EXTFN);
         if ( add.flags & XEN_PCI_DEV_VIRTFN )
         {
             pdev_info.is_virtfn = 1;
             pdev_info.physfn.bus = add.physfn.bus;
             pdev_info.physfn.devfn = add.physfn.devfn;
         }
         else
             pdev_info.is_virtfn = 0;
 
         gdprintk(XENLOG_DEBUG, "PHYSDEVOP cmd=%d: domain:%d seg:%d bus:%d \
                 devfn:%d\n", cmd, hardware_domain->domain_id, add.seg, \
 add.bus, add.devfn);
         ret = acpi_setup_smmu_for_test(hardware_domain, add.seg, add.bus, add.devfn);
         break;
     }
 
     default:
         gdprintk(XENLOG_DEBUG, "PHYSDEVOP cmd=%d: not implemented\n", cmd);
         ret = -ENOSYS;
 break;
     }
 
     return ret;
  }

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
