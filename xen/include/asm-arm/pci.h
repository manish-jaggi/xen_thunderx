#ifndef __ASM_ARM_PCI_H__
#define __ASM_ARM_PCI_H__

struct arch_pci_dev {
    struct device dev;
};

#define to_pci_dev(d) container_of( \
                                   container_of(d, struct arch_pci_dev, dev), \
                                   struct pci_dev,\
                                   arch\
                                  )
#endif /* __ASM_ARM_PCI_H__ */
