# add-overlay defines the target with following naming convention:
# <base>-<board>-dtbs = base.dtb board.dtbo
#
# Combined dtb target is also generated using the fdt_overlay tool.
# dtb-y += <base>-<board>.dtb
add-overlays = $(foreach o,$1,$(foreach b,$2,$(eval $(basename $b)-$(basename $o)-dtbs = $b $o) $(basename $b)-$(basename $o).dtb))

# khaje.dtb
KHAJE_BASE_DTB += ../qcom/khaje.dtb

bengal-dtb-$(CONFIG_ARCH_KHAJE) += $(OPLUS_KHAJE_BOARDS) $(KHAJE_BASE_DTB)
bengal-overlays-dtb-$(CONFIG_ARCH_KHAJE) += $(OPLUS_KHAJE_BOARDS) $(KHAJE_BASE_DTB)
dtb-y += $(bengal-dtb-y)

# bengal.dtb
BENGAL_BASE_DTB += ../qcom/bengal.dtb

OPLUS_BENGAL_BOARDS += \
		Alpha-B4-24288-bengal-qrd-nopmi-overlay.dtbo \
		Alpha-B4-24307-bengal-qrd-nopmi-overlay.dtbo \
		Alpha-B4-24288-IR-bengal-qrd-nopmi-overlay.dtbo \
		Alpha-B4-24307-IR-bengal-qrd-nopmi-overlay.dtbo

bengal-dtb-$(CONFIG_ARCH_BENGAL) += $(OPLUS_BENGAL_BOARDS) $(BENGAL_BASE_DTB)
bengal-overlays-dtb-$(CONFIG_ARCH_BENGAL) += $(OPLUS_BENGAL_BOARDS) $(BENGAL_BASE_DTB)
dtb-y += $(bengal-dtb-y)

always-y        := $(dtb-y)
subdir-y        := $(dts-dirs)
clean-files     := *.dtb *.dtbo

