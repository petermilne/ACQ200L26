/*
 *  IBM eServer eHCA Infiniband device driver for Linux on POWER
 *
 *  module start stop, hca detection
 *
 *  Authors: Heiko J Schick <schickhj@de.ibm.com>
 *           Hoang-Nam Nguyen <hnguyen@de.ibm.com>
 *           Joachim Fenkes <fenkes@de.ibm.com>
 *
 *  Copyright (c) 2005 IBM Corporation
 *
 *  All rights reserved.
 *
 *  This source code is distributed under a dual license of GPL v2.0 and OpenIB
 *  BSD.
 *
 * OpenIB BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials
 * provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ehca_classes.h"
#include "ehca_iverbs.h"
#include "ehca_mrmw.h"
#include "ehca_tools.h"
#include "hcp_if.h"

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Christoph Raisch <raisch@de.ibm.com>");
MODULE_DESCRIPTION("IBM eServer HCA InfiniBand Device Driver");
MODULE_VERSION("SVNEHCA_0016");

int ehca_open_aqp1     = 0;
int ehca_debug_level   = 0;
int ehca_hw_level      = 0;
int ehca_nr_ports      = 2;
int ehca_use_hp_mr     = 0;
int ehca_port_act_time = 30;
int ehca_poll_all_eqs  = 1;
int ehca_static_rate   = -1;

module_param_named(open_aqp1,     ehca_open_aqp1,     int, 0);
module_param_named(debug_level,   ehca_debug_level,   int, 0);
module_param_named(hw_level,      ehca_hw_level,      int, 0);
module_param_named(nr_ports,      ehca_nr_ports,      int, 0);
module_param_named(use_hp_mr,     ehca_use_hp_mr,     int, 0);
module_param_named(port_act_time, ehca_port_act_time, int, 0);
module_param_named(poll_all_eqs,  ehca_poll_all_eqs,  int, 0);
module_param_named(static_rate,   ehca_static_rate,   int, 0);

MODULE_PARM_DESC(open_aqp1,
		 "AQP1 on startup (0: no (default), 1: yes)");
MODULE_PARM_DESC(debug_level,
		 "debug level"
		 " (0: no debug traces (default), 1: with debug traces)");
MODULE_PARM_DESC(hw_level,
		 "hardware level"
		 " (0: autosensing (default), 1: v. 0.20, 2: v. 0.21)");
MODULE_PARM_DESC(nr_ports,
		 "number of connected ports (default: 2)");
MODULE_PARM_DESC(use_hp_mr,
		 "high performance MRs (0: no (default), 1: yes)");
MODULE_PARM_DESC(port_act_time,
		 "time to wait for port activation (default: 30 sec)");
MODULE_PARM_DESC(poll_all_eqs,
		 "polls all event queues periodically"
		 " (0: no, 1: yes (default))");
MODULE_PARM_DESC(static_rate,
		 "set permanent static rate (default: disabled)");

spinlock_t ehca_qp_idr_lock;
spinlock_t ehca_cq_idr_lock;
DEFINE_IDR(ehca_qp_idr);
DEFINE_IDR(ehca_cq_idr);

static struct list_head shca_list; /* list of all registered ehcas */
static spinlock_t shca_list_lock;

static struct timer_list poll_eqs_timer;

static int ehca_create_slab_caches(void)
{
	int ret;

	ret = ehca_init_pd_cache();
	if (ret) {
		ehca_gen_err("Cannot create PD SLAB cache.");
		return ret;
	}

	ret = ehca_init_cq_cache();
	if (ret) {
		ehca_gen_err("Cannot create CQ SLAB cache.");
		goto create_slab_caches2;
	}

	ret = ehca_init_qp_cache();
	if (ret) {
		ehca_gen_err("Cannot create QP SLAB cache.");
		goto create_slab_caches3;
	}

	ret = ehca_init_av_cache();
	if (ret) {
		ehca_gen_err("Cannot create AV SLAB cache.");
		goto create_slab_caches4;
	}

	ret = ehca_init_mrmw_cache();
	if (ret) {
		ehca_gen_err("Cannot create MR&MW SLAB cache.");
		goto create_slab_caches5;
	}

	return 0;

create_slab_caches5:
	ehca_cleanup_av_cache();

create_slab_caches4:
	ehca_cleanup_qp_cache();

create_slab_caches3:
	ehca_cleanup_cq_cache();

create_slab_caches2:
	ehca_cleanup_pd_cache();

	return ret;
}

static void ehca_destroy_slab_caches(void)
{
	ehca_cleanup_mrmw_cache();
	ehca_cleanup_av_cache();
	ehca_cleanup_qp_cache();
	ehca_cleanup_cq_cache();
	ehca_cleanup_pd_cache();
}

#define EHCA_HCAAVER  EHCA_BMASK_IBM(32,39)
#define EHCA_REVID    EHCA_BMASK_IBM(40,63)

int ehca_sense_attributes(struct ehca_shca *shca)
{
	int ret = 0;
	u64 h_ret;
	struct hipz_query_hca *rblock;

	rblock = kzalloc(H_CB_ALIGNMENT, GFP_KERNEL);
	if (!rblock) {
		ehca_gen_err("Cannot allocate rblock memory.");
		return -ENOMEM;
	}

	h_ret = hipz_h_query_hca(shca->ipz_hca_handle, rblock);
	if (h_ret != H_SUCCESS) {
		ehca_gen_err("Cannot query device properties. h_ret=%lx",
			     h_ret);
		ret = -EPERM;
		goto num_ports1;
	}

	if (ehca_nr_ports == 1)
		shca->num_ports = 1;
	else
		shca->num_ports = (u8)rblock->num_ports;

	ehca_gen_dbg(" ... found %x ports", rblock->num_ports);

	if (ehca_hw_level == 0) {
		u32 hcaaver;
		u32 revid;

		hcaaver = EHCA_BMASK_GET(EHCA_HCAAVER, rblock->hw_ver);
		revid   = EHCA_BMASK_GET(EHCA_REVID, rblock->hw_ver);

		ehca_gen_dbg(" ... hardware version=%x:%x", hcaaver, revid);

		if ((hcaaver == 1) && (revid == 0))
			shca->hw_level = 0;
		else if ((hcaaver == 1) && (revid == 1))
			shca->hw_level = 1;
		else if ((hcaaver == 1) && (revid == 2))
			shca->hw_level = 2;
	}
	ehca_gen_dbg(" ... hardware level=%x", shca->hw_level);

	shca->sport[0].rate = IB_RATE_30_GBPS;
	shca->sport[1].rate = IB_RATE_30_GBPS;

num_ports1:
	kfree(rblock);
	return ret;
}

static int init_node_guid(struct ehca_shca *shca)
{
	int ret = 0;
	struct hipz_query_hca *rblock;

	rblock = kzalloc(H_CB_ALIGNMENT, GFP_KERNEL);
	if (!rblock) {
		ehca_err(&shca->ib_device, "Can't allocate rblock memory.");
		return -ENOMEM;
	}

	if (hipz_h_query_hca(shca->ipz_hca_handle, rblock) != H_SUCCESS) {
		ehca_err(&shca->ib_device, "Can't query device properties");
		ret = -EINVAL;
		goto init_node_guid1;
	}

	memcpy(&shca->ib_device.node_guid, &rblock->node_guid, sizeof(u64));

init_node_guid1:
	kfree(rblock);
	return ret;
}

int ehca_register_device(struct ehca_shca *shca)
{
	int ret;

	ret = init_node_guid(shca);
	if (ret)
		return ret;

	strlcpy(shca->ib_device.name, "ehca%d", IB_DEVICE_NAME_MAX);
	shca->ib_device.owner               = THIS_MODULE;

	shca->ib_device.uverbs_abi_ver	    = 5;
	shca->ib_device.uverbs_cmd_mask	    =
		(1ull << IB_USER_VERBS_CMD_GET_CONTEXT)		|
		(1ull << IB_USER_VERBS_CMD_QUERY_DEVICE)	|
		(1ull << IB_USER_VERBS_CMD_QUERY_PORT)		|
		(1ull << IB_USER_VERBS_CMD_ALLOC_PD)		|
		(1ull << IB_USER_VERBS_CMD_DEALLOC_PD)		|
		(1ull << IB_USER_VERBS_CMD_REG_MR)		|
		(1ull << IB_USER_VERBS_CMD_DEREG_MR)		|
		(1ull << IB_USER_VERBS_CMD_CREATE_COMP_CHANNEL)	|
		(1ull << IB_USER_VERBS_CMD_CREATE_CQ)		|
		(1ull << IB_USER_VERBS_CMD_DESTROY_CQ)		|
		(1ull << IB_USER_VERBS_CMD_CREATE_QP)		|
		(1ull << IB_USER_VERBS_CMD_MODIFY_QP)		|
		(1ull << IB_USER_VERBS_CMD_QUERY_QP)		|
		(1ull << IB_USER_VERBS_CMD_DESTROY_QP)		|
		(1ull << IB_USER_VERBS_CMD_ATTACH_MCAST)	|
		(1ull << IB_USER_VERBS_CMD_DETACH_MCAST);

	shca->ib_device.node_type           = RDMA_NODE_IB_CA;
	shca->ib_device.phys_port_cnt       = shca->num_ports;
	shca->ib_device.dma_device          = &shca->ibmebus_dev->ofdev.dev;
	shca->ib_device.query_device        = ehca_query_device;
	shca->ib_device.query_port          = ehca_query_port;
	shca->ib_device.query_gid           = ehca_query_gid;
	shca->ib_device.query_pkey          = ehca_query_pkey;
	/* shca->in_device.modify_device    = ehca_modify_device    */
	shca->ib_device.modify_port         = ehca_modify_port;
	shca->ib_device.alloc_ucontext      = ehca_alloc_ucontext;
	shca->ib_device.dealloc_ucontext    = ehca_dealloc_ucontext;
	shca->ib_device.alloc_pd            = ehca_alloc_pd;
	shca->ib_device.dealloc_pd          = ehca_dealloc_pd;
	shca->ib_device.create_ah	    = ehca_create_ah;
	/* shca->ib_device.modify_ah	    = ehca_modify_ah;	    */
	shca->ib_device.query_ah	    = ehca_query_ah;
	shca->ib_device.destroy_ah	    = ehca_destroy_ah;
	shca->ib_device.create_qp	    = ehca_create_qp;
	shca->ib_device.modify_qp	    = ehca_modify_qp;
	shca->ib_device.query_qp	    = ehca_query_qp;
	shca->ib_device.destroy_qp	    = ehca_destroy_qp;
	shca->ib_device.post_send	    = ehca_post_send;
	shca->ib_device.post_recv	    = ehca_post_recv;
	shca->ib_device.create_cq	    = ehca_create_cq;
	shca->ib_device.destroy_cq	    = ehca_destroy_cq;
	shca->ib_device.resize_cq	    = ehca_resize_cq;
	shca->ib_device.poll_cq		    = ehca_poll_cq;
	/* shca->ib_device.peek_cq	    = ehca_peek_cq;	    */
	shca->ib_device.req_notify_cq	    = ehca_req_notify_cq;
	/* shca->ib_device.req_ncomp_notif  = ehca_req_ncomp_notif; */
	shca->ib_device.get_dma_mr	    = ehca_get_dma_mr;
	shca->ib_device.reg_phys_mr	    = ehca_reg_phys_mr;
	shca->ib_device.reg_user_mr	    = ehca_reg_user_mr;
	shca->ib_device.query_mr	    = ehca_query_mr;
	shca->ib_device.dereg_mr	    = ehca_dereg_mr;
	shca->ib_device.rereg_phys_mr	    = ehca_rereg_phys_mr;
	shca->ib_device.alloc_mw	    = ehca_alloc_mw;
	shca->ib_device.bind_mw		    = ehca_bind_mw;
	shca->ib_device.dealloc_mw	    = ehca_dealloc_mw;
	shca->ib_device.alloc_fmr	    = ehca_alloc_fmr;
	shca->ib_device.map_phys_fmr	    = ehca_map_phys_fmr;
	shca->ib_device.unmap_fmr	    = ehca_unmap_fmr;
	shca->ib_device.dealloc_fmr	    = ehca_dealloc_fmr;
	shca->ib_device.attach_mcast	    = ehca_attach_mcast;
	shca->ib_device.detach_mcast	    = ehca_detach_mcast;
	/* shca->ib_device.process_mad	    = ehca_process_mad;	    */
	shca->ib_device.mmap		    = ehca_mmap;

	ret = ib_register_device(&shca->ib_device);
	if (ret)
		ehca_err(&shca->ib_device,
			 "ib_register_device() failed ret=%x", ret);

	return ret;
}

static int ehca_create_aqp1(struct ehca_shca *shca, u32 port)
{
	struct ehca_sport *sport = &shca->sport[port - 1];
	struct ib_cq *ibcq;
	struct ib_qp *ibqp;
	struct ib_qp_init_attr qp_init_attr;
	int ret;

	if (sport->ibcq_aqp1) {
		ehca_err(&shca->ib_device, "AQP1 CQ is already created.");
		return -EPERM;
	}

	ibcq = ib_create_cq(&shca->ib_device, NULL, NULL, (void*)(-1), 10);
	if (IS_ERR(ibcq)) {
		ehca_err(&shca->ib_device, "Cannot create AQP1 CQ.");
		return PTR_ERR(ibcq);
	}
	sport->ibcq_aqp1 = ibcq;

	if (sport->ibqp_aqp1) {
		ehca_err(&shca->ib_device, "AQP1 QP is already created.");
		ret = -EPERM;
		goto create_aqp1;
	}

	memset(&qp_init_attr, 0, sizeof(struct ib_qp_init_attr));
	qp_init_attr.send_cq          = ibcq;
	qp_init_attr.recv_cq          = ibcq;
	qp_init_attr.sq_sig_type      = IB_SIGNAL_ALL_WR;
	qp_init_attr.cap.max_send_wr  = 100;
	qp_init_attr.cap.max_recv_wr  = 100;
	qp_init_attr.cap.max_send_sge = 2;
	qp_init_attr.cap.max_recv_sge = 1;
	qp_init_attr.qp_type          = IB_QPT_GSI;
	qp_init_attr.port_num         = port;
	qp_init_attr.qp_context       = NULL;
	qp_init_attr.event_handler    = NULL;
	qp_init_attr.srq              = NULL;

	ibqp = ib_create_qp(&shca->pd->ib_pd, &qp_init_attr);
	if (IS_ERR(ibqp)) {
		ehca_err(&shca->ib_device, "Cannot create AQP1 QP.");
		ret = PTR_ERR(ibqp);
		goto create_aqp1;
	}
	sport->ibqp_aqp1 = ibqp;

	return 0;

create_aqp1:
	ib_destroy_cq(sport->ibcq_aqp1);
	return ret;
}

static int ehca_destroy_aqp1(struct ehca_sport *sport)
{
	int ret;

	ret = ib_destroy_qp(sport->ibqp_aqp1);
	if (ret) {
		ehca_gen_err("Cannot destroy AQP1 QP. ret=%x", ret);
		return ret;
	}

	ret = ib_destroy_cq(sport->ibcq_aqp1);
	if (ret)
		ehca_gen_err("Cannot destroy AQP1 CQ. ret=%x", ret);

	return ret;
}

static ssize_t ehca_show_debug_level(struct device_driver *ddp, char *buf)
{
	return  snprintf(buf, PAGE_SIZE, "%d\n",
			 ehca_debug_level);
}

static ssize_t ehca_store_debug_level(struct device_driver *ddp,
				      const char *buf, size_t count)
{
	int value = (*buf) - '0';
	if (value >= 0 && value <= 9)
		ehca_debug_level = value;
	return 1;
}

DRIVER_ATTR(debug_level, S_IRUSR | S_IWUSR,
	    ehca_show_debug_level, ehca_store_debug_level);

void ehca_create_driver_sysfs(struct ibmebus_driver *drv)
{
	driver_create_file(&drv->driver, &driver_attr_debug_level);
}

void ehca_remove_driver_sysfs(struct ibmebus_driver *drv)
{
	driver_remove_file(&drv->driver, &driver_attr_debug_level);
}

#define EHCA_RESOURCE_ATTR(name)                                           \
static ssize_t  ehca_show_##name(struct device *dev,                       \
				 struct device_attribute *attr,            \
				 char *buf)                                \
{									   \
	struct ehca_shca *shca;						   \
	struct hipz_query_hca *rblock;				           \
	int data;                                                          \
									   \
	shca = dev->driver_data;					   \
									   \
	rblock = kzalloc(H_CB_ALIGNMENT, GFP_KERNEL);			   \
	if (!rblock) {						           \
		dev_err(dev, "Can't allocate rblock memory.");		   \
		return 0;						   \
	}								   \
									   \
	if (hipz_h_query_hca(shca->ipz_hca_handle, rblock) != H_SUCCESS) { \
		dev_err(dev, "Can't query device properties");	   	   \
		kfree(rblock);					   	   \
		return 0;					   	   \
	}								   \
									   \
	data = rblock->name;                                               \
	kfree(rblock);                                                     \
									   \
	if ((strcmp(#name, "num_ports") == 0) && (ehca_nr_ports == 1))	   \
		return snprintf(buf, 256, "1\n");			   \
	else								   \
		return snprintf(buf, 256, "%d\n", data);		   \
									   \
}									   \
static DEVICE_ATTR(name, S_IRUGO, ehca_show_##name, NULL);

EHCA_RESOURCE_ATTR(num_ports);
EHCA_RESOURCE_ATTR(hw_ver);
EHCA_RESOURCE_ATTR(max_eq);
EHCA_RESOURCE_ATTR(cur_eq);
EHCA_RESOURCE_ATTR(max_cq);
EHCA_RESOURCE_ATTR(cur_cq);
EHCA_RESOURCE_ATTR(max_qp);
EHCA_RESOURCE_ATTR(cur_qp);
EHCA_RESOURCE_ATTR(max_mr);
EHCA_RESOURCE_ATTR(cur_mr);
EHCA_RESOURCE_ATTR(max_mw);
EHCA_RESOURCE_ATTR(cur_mw);
EHCA_RESOURCE_ATTR(max_pd);
EHCA_RESOURCE_ATTR(max_ah);

static ssize_t ehca_show_adapter_handle(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct ehca_shca *shca = dev->driver_data;

	return sprintf(buf, "%lx\n", shca->ipz_hca_handle.handle);

}
static DEVICE_ATTR(adapter_handle, S_IRUGO, ehca_show_adapter_handle, NULL);


void ehca_create_device_sysfs(struct ibmebus_dev *dev)
{
	device_create_file(&dev->ofdev.dev, &dev_attr_adapter_handle);
	device_create_file(&dev->ofdev.dev, &dev_attr_num_ports);
	device_create_file(&dev->ofdev.dev, &dev_attr_hw_ver);
	device_create_file(&dev->ofdev.dev, &dev_attr_max_eq);
	device_create_file(&dev->ofdev.dev, &dev_attr_cur_eq);
	device_create_file(&dev->ofdev.dev, &dev_attr_max_cq);
	device_create_file(&dev->ofdev.dev, &dev_attr_cur_cq);
	device_create_file(&dev->ofdev.dev, &dev_attr_max_qp);
	device_create_file(&dev->ofdev.dev, &dev_attr_cur_qp);
	device_create_file(&dev->ofdev.dev, &dev_attr_max_mr);
	device_create_file(&dev->ofdev.dev, &dev_attr_cur_mr);
	device_create_file(&dev->ofdev.dev, &dev_attr_max_mw);
	device_create_file(&dev->ofdev.dev, &dev_attr_cur_mw);
	device_create_file(&dev->ofdev.dev, &dev_attr_max_pd);
	device_create_file(&dev->ofdev.dev, &dev_attr_max_ah);
}

void ehca_remove_device_sysfs(struct ibmebus_dev *dev)
{
	device_remove_file(&dev->ofdev.dev, &dev_attr_adapter_handle);
	device_remove_file(&dev->ofdev.dev, &dev_attr_num_ports);
	device_remove_file(&dev->ofdev.dev, &dev_attr_hw_ver);
	device_remove_file(&dev->ofdev.dev, &dev_attr_max_eq);
	device_remove_file(&dev->ofdev.dev, &dev_attr_cur_eq);
	device_remove_file(&dev->ofdev.dev, &dev_attr_max_cq);
	device_remove_file(&dev->ofdev.dev, &dev_attr_cur_cq);
	device_remove_file(&dev->ofdev.dev, &dev_attr_max_qp);
	device_remove_file(&dev->ofdev.dev, &dev_attr_cur_qp);
	device_remove_file(&dev->ofdev.dev, &dev_attr_max_mr);
	device_remove_file(&dev->ofdev.dev, &dev_attr_cur_mr);
	device_remove_file(&dev->ofdev.dev, &dev_attr_max_mw);
	device_remove_file(&dev->ofdev.dev, &dev_attr_cur_mw);
	device_remove_file(&dev->ofdev.dev, &dev_attr_max_pd);
	device_remove_file(&dev->ofdev.dev, &dev_attr_max_ah);
}

static int __devinit ehca_probe(struct ibmebus_dev *dev,
				const struct of_device_id *id)
{
	struct ehca_shca *shca;
	u64 *handle;
	struct ib_pd *ibpd;
	int ret;

	handle = (u64 *)get_property(dev->ofdev.node, "ibm,hca-handle", NULL);
	if (!handle) {
		ehca_gen_err("Cannot get eHCA handle for adapter: %s.",
			     dev->ofdev.node->full_name);
		return -ENODEV;
	}

	if (!(*handle)) {
		ehca_gen_err("Wrong eHCA handle for adapter: %s.",
			     dev->ofdev.node->full_name);
		return -ENODEV;
	}

	shca = (struct ehca_shca *)ib_alloc_device(sizeof(*shca));
	if (!shca) {
		ehca_gen_err("Cannot allocate shca memory.");
		return -ENOMEM;
	}

	shca->ibmebus_dev = dev;
	shca->ipz_hca_handle.handle = *handle;
	dev->ofdev.dev.driver_data = shca;

	ret = ehca_sense_attributes(shca);
	if (ret < 0) {
		ehca_gen_err("Cannot sense eHCA attributes.");
		goto probe1;
	}

	ret = ehca_register_device(shca);
	if (ret) {
		ehca_gen_err("Cannot register Infiniband device");
		goto probe1;
	}

	/* create event queues */
	ret = ehca_create_eq(shca, &shca->eq, EHCA_EQ, 2048);
	if (ret) {
		ehca_err(&shca->ib_device, "Cannot create EQ.");
		goto probe2;
	}

	ret = ehca_create_eq(shca, &shca->neq, EHCA_NEQ, 513);
	if (ret) {
		ehca_err(&shca->ib_device, "Cannot create NEQ.");
		goto probe3;
	}

	/* create internal protection domain */
	ibpd = ehca_alloc_pd(&shca->ib_device, (void*)(-1), NULL);
	if (IS_ERR(ibpd)) {
		ehca_err(&shca->ib_device, "Cannot create internal PD.");
		ret = PTR_ERR(ibpd);
		goto probe4;
	}

	shca->pd = container_of(ibpd, struct ehca_pd, ib_pd);
	shca->pd->ib_pd.device = &shca->ib_device;

	/* create internal max MR */
	ret = ehca_reg_internal_maxmr(shca, shca->pd, &shca->maxmr);

	if (ret) {
		ehca_err(&shca->ib_device, "Cannot create internal MR ret=%x",
			 ret);
		goto probe5;
	}

	/* create AQP1 for port 1 */
	if (ehca_open_aqp1 == 1) {
		shca->sport[0].port_state = IB_PORT_DOWN;
		ret = ehca_create_aqp1(shca, 1);
		if (ret) {
			ehca_err(&shca->ib_device,
				 "Cannot create AQP1 for port 1.");
			goto probe6;
		}
	}

	/* create AQP1 for port 2 */
	if ((ehca_open_aqp1 == 1) && (shca->num_ports == 2)) {
		shca->sport[1].port_state = IB_PORT_DOWN;
		ret = ehca_create_aqp1(shca, 2);
		if (ret) {
			ehca_err(&shca->ib_device,
				 "Cannot create AQP1 for port 2.");
			goto probe7;
		}
	}

	ehca_create_device_sysfs(dev);

	spin_lock(&shca_list_lock);
	list_add(&shca->shca_list, &shca_list);
	spin_unlock(&shca_list_lock);

	return 0;

probe7:
	ret = ehca_destroy_aqp1(&shca->sport[0]);
	if (ret)
		ehca_err(&shca->ib_device,
			 "Cannot destroy AQP1 for port 1. ret=%x", ret);

probe6:
	ret = ehca_dereg_internal_maxmr(shca);
	if (ret)
		ehca_err(&shca->ib_device,
			 "Cannot destroy internal MR. ret=%x", ret);

probe5:
	ret = ehca_dealloc_pd(&shca->pd->ib_pd);
	if (ret)
		ehca_err(&shca->ib_device,
			 "Cannot destroy internal PD. ret=%x", ret);

probe4:
	ret = ehca_destroy_eq(shca, &shca->neq);
	if (ret)
		ehca_err(&shca->ib_device,
			 "Cannot destroy NEQ. ret=%x", ret);

probe3:
	ret = ehca_destroy_eq(shca, &shca->eq);
	if (ret)
		ehca_err(&shca->ib_device,
			 "Cannot destroy EQ. ret=%x", ret);

probe2:
	ib_unregister_device(&shca->ib_device);

probe1:
	ib_dealloc_device(&shca->ib_device);

	return -EINVAL;
}

static int __devexit ehca_remove(struct ibmebus_dev *dev)
{
	struct ehca_shca *shca = dev->ofdev.dev.driver_data;
	int ret;

	ehca_remove_device_sysfs(dev);

	if (ehca_open_aqp1 == 1) {
		int i;
		for (i = 0; i < shca->num_ports; i++) {
			ret = ehca_destroy_aqp1(&shca->sport[i]);
			if (ret)
				ehca_err(&shca->ib_device,
					 "Cannot destroy AQP1 for port %x "
					 "ret=%x", ret, i);
		}
	}

	ib_unregister_device(&shca->ib_device);

	ret = ehca_dereg_internal_maxmr(shca);
	if (ret)
		ehca_err(&shca->ib_device,
			 "Cannot destroy internal MR. ret=%x", ret);

	ret = ehca_dealloc_pd(&shca->pd->ib_pd);
	if (ret)
		ehca_err(&shca->ib_device,
			 "Cannot destroy internal PD. ret=%x", ret);

	ret = ehca_destroy_eq(shca, &shca->eq);
	if (ret)
		ehca_err(&shca->ib_device, "Cannot destroy EQ. ret=%x", ret);

	ret = ehca_destroy_eq(shca, &shca->neq);
	if (ret)
		ehca_err(&shca->ib_device, "Canot destroy NEQ. ret=%x", ret);

	ib_dealloc_device(&shca->ib_device);

	spin_lock(&shca_list_lock);
	list_del(&shca->shca_list);
	spin_unlock(&shca_list_lock);

	return ret;
}

static struct of_device_id ehca_device_table[] =
{
	{
		.name       = "lhca",
		.compatible = "IBM,lhca",
	},
	{},
};

static struct ibmebus_driver ehca_driver = {
	.name     = "ehca",
	.id_table = ehca_device_table,
	.probe    = ehca_probe,
	.remove   = ehca_remove,
};

void ehca_poll_eqs(unsigned long data)
{
	struct ehca_shca *shca;

	spin_lock(&shca_list_lock);
	list_for_each_entry(shca, &shca_list, shca_list) {
		if (shca->eq.is_initialized)
			ehca_tasklet_eq((unsigned long)(void*)shca);
	}
	mod_timer(&poll_eqs_timer, jiffies + HZ);
	spin_unlock(&shca_list_lock);
}

int __init ehca_module_init(void)
{
	int ret;

	printk(KERN_INFO "eHCA Infiniband Device Driver "
	                 "(Rel.: SVNEHCA_0016)\n");
	idr_init(&ehca_qp_idr);
	idr_init(&ehca_cq_idr);
	spin_lock_init(&ehca_qp_idr_lock);
	spin_lock_init(&ehca_cq_idr_lock);

	INIT_LIST_HEAD(&shca_list);
	spin_lock_init(&shca_list_lock);

	if ((ret = ehca_create_comp_pool())) {
		ehca_gen_err("Cannot create comp pool.");
		return ret;
	}

	if ((ret = ehca_create_slab_caches())) {
		ehca_gen_err("Cannot create SLAB caches");
		ret = -ENOMEM;
		goto module_init1;
	}

	if ((ret = ibmebus_register_driver(&ehca_driver))) {
		ehca_gen_err("Cannot register eHCA device driver");
		ret = -EINVAL;
		goto module_init2;
	}

	ehca_create_driver_sysfs(&ehca_driver);

	if (ehca_poll_all_eqs != 1) {
		ehca_gen_err("WARNING!!!");
		ehca_gen_err("It is possible to lose interrupts.");
	} else {
		init_timer(&poll_eqs_timer);
		poll_eqs_timer.function = ehca_poll_eqs;
		poll_eqs_timer.expires = jiffies + HZ;
		add_timer(&poll_eqs_timer);
	}

	return 0;

module_init2:
	ehca_destroy_slab_caches();

module_init1:
	ehca_destroy_comp_pool();
	return ret;
};

void __exit ehca_module_exit(void)
{
	if (ehca_poll_all_eqs == 1)
		del_timer_sync(&poll_eqs_timer);

	ehca_remove_driver_sysfs(&ehca_driver);
	ibmebus_unregister_driver(&ehca_driver);

	ehca_destroy_slab_caches();

	ehca_destroy_comp_pool();

	idr_destroy(&ehca_cq_idr);
	idr_destroy(&ehca_qp_idr);
};

module_init(ehca_module_init);
module_exit(ehca_module_exit);