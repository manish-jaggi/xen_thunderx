This repository is a fork of xen 4.11 release.
It will contain patches for running xen hypervisor on thunderx socs.

Update firmware for TX2 

Building Xen - Natively for thunderx2 
----------------------------------------
1. export XEN_CONFIG_EXPERT=y
2. Enable the following in xen menuconfig

CONFIG_64BIT=y
CONFIG_ARM_64=y
CONFIG_ARM=y
CONFIG_NR_CPUS=256
CONFIG_ACPI=y
CONFIG_HAS_GICV3=y
CONFIG_HAS_ITS=y

Do not set 
CONFIG_NEW_VGIC and 
CONFIG_SBSA_VUART_CONSOLE 

3. Compile Xen with
$ make dist-xen   -j222 CONFIG_EARLY_PRINTK=pl011,0x0402020000

4. Build tools 
- apply psr_mode_workaround.patch 
- build tools 
- copy files from dist/install into filesystems root directory.

Compiling Dom0 kernel
---------------------
- Use linux_config to compile linux kernel. It is based on 4.19-rc3

Booting via UEFI shell
------------------------- 
1. Copy xen/xen in /boot/efi folder
2. Copy kenrel image in /boot/efi
3. Create xen.cfg 

[xen]
options=no-bootscrub iommu=no dom0_mem=1G acpi=force  dom0_max_vcpus=8 
kernel=Image console=hvc0 debug=y rw root=/dev/sda5 

Booting Xen
----------------
1. On UEFI shell type 
	xen -cfg=xen.cfg
2. On Dom0 Linux shell init the xencommons
- export LD_LIBRARY_PATH=/usr/local/lib
- /etc/init.d/xencommons start

3. Running Domu, use the guest.cfg
kernel = "Image" 
name = "domu2"
memory = 4096
vcpus = 4
extra = "console=hvc0 root=/dev/xvda rw"
disk = [ 'phy:/dev/loop1,xvda,w' ]

$ xl create guest.cfg
$ xl console domu2
   

