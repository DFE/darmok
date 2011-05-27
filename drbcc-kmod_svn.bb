DESCRIPTION = "HydraIP Board Controller Kernel Module"
SECTION = "kernel/modules"
RDEPENDS = "kernel (${KERNEL_VERSION})"
DEPENDS = "virtual/kernel"

SRCREV = "${AUTOREV}" 
PR = "r0"

SRC_URI="svn://dresearchfe.jira.com/svn/HYP/trunk/LINUX;module=drbcc-kmod;proto=https"

PV = "svnr${SRCREV}"
S = "${WORKDIR}/drbcc-kmod"

inherit module

do_install() {
	install -d ${D}/lib/modules/${KERNEL_VERSION}/
	install -m 0644 drbcc${KERNEL_OBJECT_SUFFIX} ${D}/lib/modules/${KERNEL_VERSION}/
}
