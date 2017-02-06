dts-dirs += apm
dts-dirs += arm
dts-dirs += cavium
ifndef CONFIG_TCT_8996_COMMON
dts-dirs += qcom
else
dts-dirs += idol4s_cn
endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
