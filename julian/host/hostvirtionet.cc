/** @file
 * Virtio Net
 *
 * Copyright (C) 2013, Julian Stecklina <jsteckli@os.inf.tu-dresden.de>
 * Economic rights: Technische Universitaet Dresden (Germany)
 *
 * This file is part of NUL.
 *
 * NUL is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * NUL is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public
 * License version 2 for more details.
 */

#include <nul/types.h>
#include <nul/motherboard.h>
#include <host/hostpci.h>
#include <host/jsdriver.h>

/* A 32-bit r/o bitmask of the features supported by the host */
#define VIRTIO_PCI_HOST_FEATURES	0

/* A 32-bit r/w bitmask of features activated by the guest */
#define VIRTIO_PCI_GUEST_FEATURES	4

/* A 32-bit r/w PFN for the currently selected queue */
#define VIRTIO_PCI_QUEUE_PFN		8

/* A 16-bit r/o queue size for the currently selected queue */
#define VIRTIO_PCI_QUEUE_NUM		12

/* A 16-bit r/w queue selector */
#define VIRTIO_PCI_QUEUE_SEL		14

/* A 16-bit r/w queue notifier */
#define VIRTIO_PCI_QUEUE_NOTIFY		16

/* An 8-bit device status register.  */
#define VIRTIO_PCI_STATUS		18


/* An 8-bit r/o interrupt status register.  Reading the value will return the
 * current contents of the ISR and will also clear it.  This is effectively
 * a read-and-acknowledge. */
#define VIRTIO_PCI_ISR			19

/* A 16-bit vector for selected queue notifications. */
#define VIRTIO_MSI_QUEUE_VECTOR         22


static inline unsigned vring_size(unsigned int num)
{
  const unsigned align = 4096;
  return ((16 * num + 2 * (3 + num) + align - 1) & ~(align - 1))
    + 2 * 3 + 8 * num;
}



class HostVirtioNet : public PciDriver,
                      public StaticReceiver<HostVirtioNet>
{
private:
  static const unsigned QUEUES       = 2;
  static const unsigned MEASUREMENTS = 100000;

  #include "host/simplehwioin.h"
  #include "host/simplehwioout.h"

  HostPci  _pci;
  uint16   _ioport;
  unsigned _hostirq[QUEUES];

  uint64   _last_time;
  unsigned _count;

  char     *_vring[QUEUES];
  //const char *debug_getname() { return "HostVirtioNet"; }

  uint64   _times[MEASUREMENTS];

public:

  void bench_notify()
  {
    if (_count >= MEASUREMENTS) {
      if (_count == MEASUREMENTS) {
        for (unsigned i = 0; i < MEASUREMENTS; i++)
          Logging::printf("RTT %llu\n", _times[i]);
        _count++;
      }

      return;
    }

    if (_last_time != 0)
      _times[_count++] = _clock->time() - _last_time;

    _last_time = _clock->time();

    // Notify queue zero
    outw(0, _ioport + VIRTIO_PCI_QUEUE_NOTIFY);

  }

  void enable_irqs()
  {
    for (unsigned i = 0; i < QUEUES; i++) {
      // Configure MSI-X
      _hostirq[i] = _pci.get_gsi_msi(_bus_hostop, _bdf, i);
      msg(PCI, "Queue %u -> IRQ%u\n", i, _hostirq[i]);

      outw(i, _ioport + VIRTIO_PCI_QUEUE_SEL);
      outw(i, _ioport + VIRTIO_MSI_QUEUE_VECTOR);
    }

    outb(7 , _ioport + VIRTIO_PCI_STATUS);

    //bench_notify();
  }

  bool receive(MessageIrq &irq_msg)
  {
    for (unsigned i = 0; i < QUEUES; i++) {
      if (_hostirq[i] == irq_msg.line) {
        //msg(PCI, "IRQ%u! %x\n", irq_msg.line, inb(_ioport + VIRTIO_PCI_ISR));

        // Keep benchmarking
        if (i == 0) bench_notify();

        return true;
      }
    }

    return false;
  }

  HostVirtioNet(HostPci pci, DBus<MessageHostOp> &bus_hostop, Clock *clock, unsigned bdf,
                DBus<MessageHwIOIn> &bus_hwioin, DBus<MessageHwIOOut> &bus_hwioout)
    : PciDriver("VirtioNet", bus_hostop, clock, ALL & ~IRQ, bdf),
      _bus_hwioin(bus_hwioin), _bus_hwioout(bus_hwioout),
      _pci(pci), _last_time(0), _count(0)
  {
    msg(INFO, "Found Intel Virtio network device at %x.\n", bdf);

    for (unsigned bar_i = 0; bar_i < pci.MAX_BAR; bar_i++) {
      uint32 bar_addr = pci.BAR0 + bar_i;
      uint32 bar = pci.conf_read(_bdf, bar_addr);
      uint64 size  = pci.bar_size(_bdf, bar_addr);

      msg(PCI, "BAR %u: %08x (size %08llx)\n", bar_i, bar, size);

      // Skip non-IO BARs
      if ((bar & pci.BAR_IO) != 1) continue;

      // Memory BAR
      // XXX 64-bit bars!
      uint16 io_addr = bar & pci.BAR_IO_MASK;
      msg(PCI, "IO %x\n", io_addr);
      _ioport = io_addr;

      MessageHostOp msg1(MessageHostOp::OP_ALLOC_IOIO_REGION, (_ioport << 8) | 5);
      if (!_bus_hostop.send(msg1))
        Logging::panic("%s failed to allocate ports\n", __PRETTY_FUNCTION__);

      outl(inl(_ioport + VIRTIO_PCI_HOST_FEATURES),
           _ioport + VIRTIO_PCI_GUEST_FEATURES);

      for (unsigned i = 0; i < QUEUES; i++) {
        outw(i, _ioport + VIRTIO_PCI_QUEUE_SEL);

        uint32 size = inl(_ioport + VIRTIO_PCI_QUEUE_NUM);
        _vring[i] = new (4096) char[vring_size(size)];
        outl(addr2phys(_vring[i]) >> 12, _ioport + VIRTIO_PCI_QUEUE_PFN);
      }

      break;
    }

  }
};

PARAM_HANDLER(hostvirtionet,
	      "hostvirtionet:instance - provide driver for Virtio network device.",
	      "Example: virtionet:0")
{
  HostPci pci(mb.bus_hwpcicfg, mb.bus_hostop);
  unsigned found = 0;

  for (unsigned bdf, num = 0; (bdf = pci.search_device(0x2, 0x0, num++));) {
    unsigned cfg0 = pci.conf_read(bdf, 0x0);
    if (cfg0 == 0x10001af4) {
      if (found++ == argv[0]) {
        HostVirtioNet *dev = new HostVirtioNet(pci, mb.bus_hostop, mb.clock(), bdf,
                                               mb.bus_hwioin, mb.bus_hwioout);
        mb.bus_hostirq.add(dev, &HostVirtioNet::receive_static<MessageIrq>);
        dev->enable_irqs();
      }
    }
  }
}

// EOF
