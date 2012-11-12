DESCRIPTION = "HydraIP Board Controller Kernel Module"
SECTION = "kernel/modules"
RDEPENDS = "kernel (${KERNEL_VERSION})"
DEPENDS = "virtual/kernel"

PR = "r0"

SRC_URI="file://${PN}-sources"

PV = "svnr${@svn_revision(d)}"
S = "${WORKDIR}/${PN}-sources"

inherit module svn-helper

do_install() {
	install -d ${D}/lib/modules/${KERNEL_VERSION}/
	install -m 0644 drbcc${KERNEL_OBJECT_SUFFIX} ${D}/lib/modules/${KERNEL_VERSION}/
}
