DESCRIPTION = "HydraIP Board Controller Kernel Module"
SECTION = "kernel/modules"
RDEPENDS = "kernel (${KERNEL_VERSION})"
DEPENDS = "virtual/kernel"

PR = "r0"
OWNSRCREV = "${@'$Rev$'.split()[1]}"

SRC_URI="file://${PN}-sources"

PV = "svnr${OWNSRCREV}"
S = "${WORKDIR}/${PN}-sources"

inherit module

do_install() {
	install -d ${D}/lib/modules/${KERNEL_VERSION}/
	install -m 0644 drbcc${KERNEL_OBJECT_SUFFIX} ${D}/lib/modules/${KERNEL_VERSION}/
}
