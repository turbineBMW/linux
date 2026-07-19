.. SPDX-License-Identifier: GPL-2.0-only

Surface Pro 11 camera support provenance
========================================

Scope
-----

This document records the origin of the Surface Pro 11 camera changes in
this tree.  It is a provenance record, not legal advice.

No Microsoft or other third-party binary, source file, debug trace, firmware
image, symbol file, or configuration file is included.  The author worked on
personally owned hardware, had ordinary end-user access only, and was not
subject to an NDA.  Windows driver and firmware file names were used to
identify relevant devices and runtime activity.  Sensor transactions and the
small number of board-specific register values retained here were independently
written from runtime observations made on that hardware.

Source classification
---------------------

``drivers/media/i2c/imx681.c`` and ``imx681-tables.h``
  Independently authored Linux driver.  The Surface Pro 11 mode table records
  runtime I2C transactions observed on the author's hardware.  It does not
  contain a copied vendor configuration table.  An unobserved table used only
  during early experiments is deliberately excluded.

``drivers/media/i2c/ov13858.c``
  Extends the existing GPL-2.0 Linux OV13858 driver with one Surface Pro 11
  mode reconstructed from runtime I2C observations.  Probe-time bus scanners
  and bring-up-only controls are deliberately excluded.

``drivers/media/i2c/vd55g0.c``, ``vd55g0_patches.h``, and the VD55G0 binding
  Imported from STMicroelectronics' GPL driver at commit
  ``9134fe572b77f906344f37ba227f375db73dc026``::

    https://github.com/STMicroelectronics/vd55g0-linux-driver

  ST's patch arrays are sensor firmware distributed by ST under GPL-2.0 in
  that repository.  They replace an earlier experimental reconstruction.

``drivers/media/platform/qcom/camss/camss-csiphy-3ph-1-0.c``
  The X1E80100 C-PHY common, lane, interrupt, and 2.5-Gsym/s data-rate values
  are adapted from Qualcomm's ``cam_csiphy_2_1_2_hwreg.h`` under
  GPL-2.0-only.  The exact source blob is
  ``347fb4944ccedfead1aa0c5260e6b41a5a038017``::

    https://github.com/LineageOS/android_kernel_oneplus_sm8550-modules/blob/lineage-23.2/qcom/opensource/camera-kernel/drivers/cam_sensor_module/cam_csiphy/include/cam_csiphy_2_1_2_hwreg.h

  Copyright for those values remains with Qualcomm Innovation Center, Inc.
  The Surface-specific lane-enable value is an independent runtime
  observation.  Private replay tables and CAMNOC comparison data are not
  included.

CAMSS and devicetree foundations
  The implementation builds on the upstream Linux CAMSS, CCI, camera clock,
  media-controller, and X1E80100 binding work.  In particular, retain credit
  for Bryan O'Donoghue and Linaro's X1E80100 CAMSS binding work.  The board
  description uses hardware enumeration, standard firmware descriptions, and
  observations made on the author's Surface Pro 11.

Denali ath12k rfkill prerequisite
  The ath12k devicetree ``disable-rfkill`` support is Dale Whinham's GPL
  contribution, originally commit
  ``d7e0b837ef4672f294af3d7f59eefe7a241371db`` with his authorship and
  Signed-off-by trailer preserved.  The matching Denali property was
  reintroduced after a one-shot hardware test showed that WCN7850 firmware
  initialized but no wireless PHY registered without the quirk.  Bluetooth
  continued to operate over its separate UART transport.

Review policy
-------------

Do not add generated dumps, extracted binary payloads, decompiled output,
private replay tables, or diagnostic traces to this branch.  New observed
hardware behavior should be expressed as ordinary Linux driver logic and
documented as an independent runtime observation.  Third-party source must
carry a compatible license, its copyright notice, and an immutable source
revision.

The code was developed with LLM assistance.  Provenance classifications and
the decision to publish remain the human maintainer's responsibility.
