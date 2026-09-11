#include <errno.h>
#include <getopt.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "rk_debug.h"
#include "rk_defines.h"
#include "rk_mpi_gdc.h"
#include "rk_mpi_mb.h"
#include "rk_mpi_sys.h"
#include "rk_mpi_venc.h"
#include "rk_mpi_vi.h"
#include "rk_mpi_vo.h"
#include "rk_mpi_vpss.h"

#include <rk_aiq_user_api2_acsm.h>
#include <rk_aiq_user_api2_camgroup.h>
#include <rk_aiq_user_api2_imgproc.h>
#include <rk_aiq_user_api2_isp.h>
#include <rk_aiq_user_api2_sysctl.h>

#define VI_DEV 0
#define ISP_ONLINE_MAIN_CHN 0
#define VPSS_ONLINE_MAIN_CHN 3
#define GDC_MAIN_CHN 0
#define GDC_DUMMY_CHN 1
#define VENC_MAIN_CHN 0

static struct {
	uint32_t input_width;
	uint32_t input_height;
	uint32_t output_width;
	uint32_t output_height;
	uint32_t display_width;
	uint32_t display_height;
	uint32_t display_rotation;
	uint32_t iio_device_idx;
	int32_t frame_count;
	uint32_t loop_count;
	uint32_t type;
	uint32_t fps;
	bool enable_compress;
	bool enable_test;
	bool enable_rkaiq;
	bool enable_amtd;
	bool enable_memc;
	bool enable_vpss;
	const char* sensor_name;
} rk_cmd_param;

static bool quit = false;
static rk_aiq_sys_ctx_t *g_aiq_ctx;

enum {
	TYPE_GDC_VO = 0,
	TYPE_GDC_VENC,
	TYPE_GDC_GET_FRAME,
	TYPE_MAX_NUM,
};

static char optstr[] = "?::w:h:c:l:t:p:W:H:r:R:P:f:";
static const struct option long_options[] = {
	{"input_width", required_argument, RK_NULL, 'w'},
	{"input_height", required_argument, RK_NULL, 'h'},
	{"output_width", required_argument, RK_NULL, 'W'},
	{"output_height", required_argument, RK_NULL, 'H'},
	{"display_width", required_argument, RK_NULL, 'd' + 'w'},
	{"display_height", required_argument, RK_NULL, 'd' + 'h'},
	{"display_rotation", required_argument, RK_NULL, 'R'},
	{"frame_count", required_argument, RK_NULL, 'c'},
	{"loop_count", required_argument, RK_NULL, 'l'},
	{"type", required_argument, RK_NULL, 't'},
	{"fps", required_argument, RK_NULL, 'f'},
	{"enable_compress", required_argument, RK_NULL, 'e' + 'c'},
	{"enable_rkaiq", required_argument, RK_NULL, 'e' + 'r'},
	{"enable_amtd", required_argument, RK_NULL, 'e' + 'a'},
	{"enable_test", required_argument, RK_NULL, 'e' + 't'},
	{"enable_vpss", required_argument, RK_NULL, 'e' + 'v' + 'p'},
	{"enable_memc", required_argument, RK_NULL, 'e' + 'm'},
	{"iio_device_idx", required_argument, RK_NULL, 'i' + 'd' + 'd'},
	{"sensor_name", required_argument, RK_NULL, 's' + 'n'},
	{"help", required_argument, RK_NULL, 'h' + 'p'},
	{RK_NULL, 0, RK_NULL, 0},
};

static void sigterm_handler(int sig) {
	fprintf(stderr, "signal %d\n", sig);
	quit = true;
}

static void print_usage(const char *name) {
	printf("usage example:\n");
	printf("\t%s -w 3840 -h 2160\n", name);
	printf("\t-w | --input_width: gdc input width, Default:3968\n");
	printf("\t-h | --input_height: gdc input height, Default:2800\n");
	printf("\t-w | --output_width: gdc output width, Default:3840\n");
	printf("\t-h | --output_height: gdc output height, Default:2160\n");
	printf("\t--display_width: video width, Default:1440\n");
	printf("\t--display_height: video height, Default:1080\n");
	printf("\t-w | --fps: video fps, Default:30\n");
	printf("\t-r | --display_rotation: display rotation, "
		   "\n\t\t0: roation 0, 1: roation 90, 2: roation 180, 3: roation 270"
		   "\n\t\tDefault:1\n");
	printf("\t-c | --frame_count: frame number of output, Default:100\n");
	printf("\t-l | --loop_count: test loop count\n");
	printf("\t-t | --type: type, 0: vi bind gdc bind vo"
		   "\n\t\t1: vi bind gdc bind venc"
		   "\n\t\t2: vi bind gdc, get/release gdc frame\n");
	printf("\t--enable_rkaiq: enable rkaiq, Default:1\n");
	printf("\t--enable_amtd: enable amtd, this funciton depend on rkaiq, Default:1\n");
	printf("\t--enable_compress: enable compress format in video, Default:1\n");
	printf("\t--enable_test: enable test for all mode, Default:0\n");
	printf("\t--enable_vpss: enable use vpss online mode, Default:1\n");
	printf("\t--enable_memc: enable memc, this funciton depend on rkaiq, Default:0\n");
	printf("\t--iio_device_idx: imu device index in path /sys/bus/iio/devices/, Default:1\n");
	printf("\t--sensor_name: camera name, Default:imx586\n");
}

static int parse_cmd_args(int argc, const char **argv) {
	int c = 0;
	rk_cmd_param.input_width = 3968;
	rk_cmd_param.input_height = 2800;
	rk_cmd_param.output_width = 3840;
	rk_cmd_param.output_height = 2160;
	rk_cmd_param.frame_count = 100;
	rk_cmd_param.loop_count = 1;
	rk_cmd_param.enable_compress = 1;
	rk_cmd_param.enable_rkaiq = 1;
	rk_cmd_param.enable_amtd = 1;
	rk_cmd_param.enable_memc = 1;
	rk_cmd_param.enable_vpss = 1;
	rk_cmd_param.display_width = 1440;
	rk_cmd_param.display_height = 1080;
	rk_cmd_param.display_rotation = ROTATION_90;
	rk_cmd_param.type = TYPE_GDC_VENC;
	rk_cmd_param.fps = 30;
	rk_cmd_param.iio_device_idx = 1;
	rk_cmd_param.sensor_name = "imx586";
	while ((c = getopt_long(argc, argv, optstr, long_options, RK_NULL)) != -1) {
		switch (c) {
		case 'w':
			rk_cmd_param.input_width = atoi(optarg);
			break;
		case 'h':
			rk_cmd_param.input_height = atoi(optarg);
			break;
		case 'W':
			rk_cmd_param.output_width = atoi(optarg);
			break;
		case 'H':
			rk_cmd_param.output_height = atoi(optarg);
			break;
		case 'd' + 'w':
			rk_cmd_param.display_width = atoi(optarg);
			break;
		case 'd' + 'h':
			rk_cmd_param.display_height = atoi(optarg);
			break;
		case 'R':
			rk_cmd_param.display_rotation = atoi(optarg);
			break;
		case 'c':
			rk_cmd_param.frame_count = atoi(optarg);
			break;
		case 'l':
			rk_cmd_param.loop_count = atoi(optarg);
			break;
		case 't':
			rk_cmd_param.type = atoi(optarg);
			break;
		case 'f':
			rk_cmd_param.fps = atoi(optarg);
			break;
		case 'e' + 'c':
			rk_cmd_param.enable_compress = atoi(optarg);
			break;
		case 'e' + 't':
			rk_cmd_param.enable_test = atoi(optarg);
			break;
		case 'e' + 'r':
			rk_cmd_param.enable_rkaiq = atoi(optarg);
			break;
		case 'e' + 'a':
			rk_cmd_param.enable_amtd = atoi(optarg);
			break;
		case 'e' + 'm':
			rk_cmd_param.enable_memc = atoi(optarg);
			break;
		case 'e' + 'v' + 'p':
			rk_cmd_param.enable_vpss = atoi(optarg);
			break;
		case 'i' + 'd' + 'd':
			rk_cmd_param.iio_device_idx = atoi(optarg);
			break;
		case 's' + 'n':
			rk_cmd_param.sensor_name = optarg;
			break;
		case 'e' + 'h':
		default:
			print_usage(argv[0]);
			return RK_FAILURE;
		}
	}

	return RK_SUCCESS;
}

static int gdc_aiq_imu_callback(RK_VOID *pUsr, GDC_SENSOR_INFO_S *pInfo) {
	RK_LOGV("time stamp = %lld, temp is %lf,"
			"gyro data is %lf %lf %lf, acc data is %lf %lf %lf",
			pInfo->s64Timestamp, pInfo->dTemp, pInfo->dGyroData[0], pInfo->dGyroData[1],
			pInfo->dGyroData[2], pInfo->dAccData[0], pInfo->dAccData[1],
			pInfo->dAccData[2]);
	AiqImuData_t imu_data = {};
	if (pInfo == NULL) {
		RK_LOGE("========== bad imu info! ===========\n");
		return GDC_INFO_CB_SUCCESS;
	}
	imu_data.s64Timestamp = pInfo->s64Timestamp;
	imu_data.dTemp = pInfo->dTemp;
	imu_data.dGyroData[0] = pInfo->dGyroData[0];
	imu_data.dGyroData[1] = pInfo->dGyroData[1];
	imu_data.dGyroData[2] = pInfo->dGyroData[2];
	imu_data.dAccData[0] = pInfo->dAccData[0];
	imu_data.dAccData[1] = pInfo->dAccData[1];
	imu_data.dAccData[2] = pInfo->dAccData[2];
	if (g_aiq_ctx == NULL) {
		RK_LOGD("========== bad aiq ctx! ===========\n");
		return GDC_INFO_CB_SUCCESS;
	}
	if (rk_cmd_param.enable_amtd) {
		int ret = rk_aiq_user_api2_amtd_setImuData(g_aiq_ctx, &imu_data);
		if (ret != 0)
			RK_LOGD("rk_aiq_user_api2_amtd_setImuData failed %#X\n", ret);
	}
	if (rk_cmd_param.enable_memc) {
		rk_aiq_uapi2_setImuData(g_aiq_ctx, &imu_data);
	}
	return GDC_INFO_CB_SUCCESS;
}

static int gdc_aiq_frame_callback(RK_VOID *pUsr, GDC_VFAME_INFO_S *pInfo) {
	if (pInfo && pInfo->pMbBlk)
		RK_MPI_MB_ReleaseMB(pInfo->pMbBlk);
	return GDC_INFO_CB_SUCCESS;
}

static int isp_init(void) {
	int cam_id = 0;
	int ret = 0;
	const char *iq_file_dir = "/etc/iqfiles";
	rk_aiq_sys_ctx_t *aiq_ctx;
	rk_aiq_static_info_t aiq_static_info;
	if (!rk_cmd_param.enable_rkaiq)
		return 0;
	rk_aiq_uapi2_sysctl_enumStaticMetasByPhyId(cam_id, &aiq_static_info);
	if (aiq_static_info.sensor_info.phyId == -1) {
		RK_LOGI("WARN: aiq_static_info.sensor_info.phyId is %d\n",
				aiq_static_info.sensor_info.phyId);
	}

	RK_LOGI("ID: %d, sensor_name is %s, iqfiles is %s\n", cam_id,
			aiq_static_info.sensor_info.sensor_name, iq_file_dir);
	ret = rk_aiq_uapi2_sysctl_preInit_scene(aiq_static_info.sensor_info.sensor_name,
											"normal", "day");
	if (ret < 0)
		RK_LOGE("%s: failed to set scene\n", aiq_static_info.sensor_info.sensor_name);

	aiq_ctx = rk_aiq_uapi2_sysctl_init(aiq_static_info.sensor_info.sensor_name,
									   iq_file_dir, NULL, NULL);
	if (aiq_ctx == NULL) {
		RK_LOGE("rk_aiq_uapi2_sysctl_init failed!\n");
		return -1;
	}
	g_aiq_ctx = aiq_ctx;

	btnr_api_attrib_t btnr_attr = {};
	rk_aiq_user_api2_btnr_GetAttrib(aiq_ctx, &btnr_attr);
	btnr_attr.stAuto.sta.swMemc.sw_btnr_swMemc_en = rk_cmd_param.enable_memc;
	btnr_attr.stMan.sta.swMemc.sw_btnr_swMemc_en = rk_cmd_param.enable_memc;
	rk_aiq_user_api2_btnr_SetAttrib(aiq_ctx, &btnr_attr);

	ret = rk_aiq_uapi2_sysctl_prepare(g_aiq_ctx, 0, 0, RK_AIQ_WORKING_MODE_NORMAL);
	if (ret) {
		RK_LOGE("rk_aiq_uapi2_sysctl_prepare failed!\n");
		return -1;
	}
	RK_LOGI("rk_aiq_uapi2_sysctl_prepare succeed\n");
	ret = rk_aiq_uapi2_sysctl_start(g_aiq_ctx);
	if (ret) {
		RK_LOGE("rk_aiq_uapi2_sysctl_start  failed\n");
		return -1;
	}

	frameRateInfo_t info;
	info.mode = OP_MANUAL;
	info.fps = rk_cmd_param.fps;
	ret = rk_aiq_uapi2_setFrameRate(g_aiq_ctx, info);
	if (ret) {
		RK_LOGE("rk_aiq_uapi2_setFrameRate failed\n");
		return -1;
	}
	return ret;
}

static int isp_deinit(void) {
	int ret = 0;
	if (!rk_cmd_param.enable_rkaiq)
		return 0;
	if (g_aiq_ctx) {
		ret = rk_aiq_uapi2_sysctl_stop(g_aiq_ctx, false);
		if (ret) {
			RK_LOGE("rk_aiq_uapi2_sysctl_stop failed!\n");
			return -1;
		}
		RK_LOGI("rk_aiq_uapi2_sysctl_stop succeed\n");
		rk_aiq_uapi2_sysctl_deinit(g_aiq_ctx);
		RK_LOGI("rk_aiq_uapi2_sysctl_deinit succeed\n");
		g_aiq_ctx = NULL;
	}
	return ret;
}

static int vi_dev_init(void) {
	int ret = 0;
	VI_DEV_ATTR_S vi_dev_attr = {};
	VI_DEV_BIND_PIPE_S vi_dev_pipe = {};
	// 0. get dev config status
	ret = RK_MPI_VI_GetDevAttr(VI_DEV, &vi_dev_attr);
	if (ret == RK_ERR_VI_NOT_CONFIG) {
		// 0-1.config dev
		ret = RK_MPI_VI_SetDevAttr(VI_DEV, &vi_dev_attr);
		if (ret != RK_SUCCESS) {
			RK_LOGE("RK_MPI_VI_SetDevAttr %x\n", ret);
			return -1;
		}
	} else {
		RK_LOGE("RK_MPI_VI_SetDevAttr already\n");
	}
	if (rk_cmd_param.enable_memc) {
		RK_MPI_VI_CtrlRawStream(false);
	} else {
		RK_MPI_VI_CtrlRawStream(true);
	}
	// 1.get dev enable status
	ret = RK_MPI_VI_GetDevIsEnable(VI_DEV);
	if (ret != RK_SUCCESS) {
		// 1-2.enable dev
		ret = RK_MPI_VI_EnableDev(VI_DEV);
		if (ret != RK_SUCCESS) {
			RK_LOGE("RK_MPI_VI_EnableDev %x\n", ret);
			return -1;
		}
		// 1-3.bind dev/pipe
		vi_dev_pipe.u32Num = VI_DEV;
		vi_dev_pipe.PipeId[0] = VI_DEV;
		ret = RK_MPI_VI_SetDevBindPipe(VI_DEV, &vi_dev_pipe);
		if (ret != RK_SUCCESS) {
			RK_LOGE("RK_MPI_VI_SetDevBindPipe %x\n", ret);
			return -1;
		}
	} else {
		RK_LOGE("RK_MPI_VI_EnableDev already\n");
	}
	RK_LOGI("vi device init success\n");
	return ret;
}

static int vi_dev_deinit(void) {
	RK_MPI_VI_DisableDev(VI_DEV);
	return 0;
}

static int vi_chn_init(void) {
	int ret;
	int width = 0;
	int height = 0;
	int buf_cnt = 12;
	int vi_main_chn = rk_cmd_param.enable_vpss ? VPSS_ONLINE_MAIN_CHN : ISP_ONLINE_MAIN_CHN;
	VI_CHN_ATTR_S vi_chn_attr;

	if (rk_cmd_param.type == TYPE_GDC_VO) {
		width = rk_cmd_param.display_width;
		height = rk_cmd_param.display_height;
	} else {
		width = rk_cmd_param.input_width;
		height = rk_cmd_param.input_height;
	}
	memset(&vi_chn_attr, 0, sizeof(vi_chn_attr));
	vi_chn_attr.stIspOpt.u32BufCount = buf_cnt;
	vi_chn_attr.stIspOpt.enMemoryType = VI_V4L2_MEMORY_TYPE_DMABUF;
	vi_chn_attr.stSize.u32Width = width;
	vi_chn_attr.stSize.u32Height = height;
	// vi --(TILE)--> fec --(FBC)--> venc
	if (rk_cmd_param.enable_compress && rk_cmd_param.type != TYPE_GDC_VO) {
		vi_chn_attr.enVideoFormat = VIDEO_FORMAT_TILE_4x4;
		vi_chn_attr.enCompressMode = COMPRESS_MODE_NONE;
	} else {
		vi_chn_attr.enVideoFormat = VIDEO_FORMAT_LINEAR;
		vi_chn_attr.enCompressMode = COMPRESS_MODE_NONE;
	}
	vi_chn_attr.enPixelFormat = RK_FMT_YUV420SP;
	vi_chn_attr.u32Depth = 0;
	vi_chn_attr.stIspOpt.bAttchFrmInfo = true;
	ret = RK_MPI_VI_SetChnAttr(VI_DEV, vi_main_chn, &vi_chn_attr);
	ret |= RK_MPI_VI_EnableChn(VI_DEV, vi_main_chn);
	if (ret) {
		RK_LOGE("ERROR: create VI main channel error! ret=%#X\n", ret);
		return ret;
	}
	RK_LOGI("vi channel init success, width %d, height %d", width, height);
	return ret;
}

static int vi_chn_deinit(void) {
	int ret = 0;
	int vi_main_chn = rk_cmd_param.enable_vpss ? VPSS_ONLINE_MAIN_CHN : ISP_ONLINE_MAIN_CHN;
	ret = RK_MPI_VI_DisableChn(VI_DEV, vi_main_chn);
	if (ret)
		RK_LOGE("ERROR: RK_MPI_VI_DisableChn VI main channel error! ret=%x\n", ret);
	return ret;
}

static int vo_init(void) {
	int ret = 0;
	int vo_width = 0;
	int vo_height = 0;
	int vo_layer = 0;
	int vo_dev_id = 0;
	int vo_chn_id = 0;
	int vo_chn_priority = 0;
	int vo_rotation = rk_cmd_param.display_rotation;
	int layer_width = 1080, layer_height = 1920;
	uint32_t disp_buf_len;
	const char *intf_type = "MIPI";
	VO_PUB_ATTR_S vo_pub_attr = {};
	VO_VIDEO_LAYER_ATTR_S vo_layer_attr = {};
	VO_CSC_S vo_csc = {};
	VO_CHN_ATTR_S vo_chn_attr = {};

	if (rk_cmd_param.type != TYPE_GDC_VO)
		return 0;
	if (!strcmp(intf_type, "MIPI"))
		vo_pub_attr.enIntfType = VO_INTF_MIPI;
	else {
		RK_LOGE("unsupport intf type %s\n", intf_type);
		return RK_FAILURE;
	}
	if (vo_rotation == ROTATION_90 || vo_rotation == ROTATION_270) {
		vo_width = rk_cmd_param.display_height;
		vo_height = rk_cmd_param.display_width;
	} else {
		vo_width = rk_cmd_param.display_width;
		vo_height = rk_cmd_param.display_height;
	}
	vo_pub_attr.enIntfSync = VO_OUTPUT_DEFAULT;

	ret = RK_MPI_VO_SetPubAttr(vo_dev_id, &vo_pub_attr);
	ret = RK_MPI_VO_Enable(vo_dev_id);
	ret = RK_MPI_VO_GetLayerDispBufLen(vo_layer, &disp_buf_len);
	RK_LOGI("Get vo_layer %d disp buf len is %d.\n", vo_layer, disp_buf_len);
	disp_buf_len = 3;
	ret = RK_MPI_VO_SetLayerDispBufLen(vo_layer, disp_buf_len);
	RK_LOGI("Agin Get vo_layer %d disp buf len is %d.\n", vo_layer, disp_buf_len);

	ret = RK_MPI_VO_GetPubAttr(vo_dev_id, &vo_pub_attr);
	if ((vo_pub_attr.stSyncInfo.u16Hact == 0) || (vo_pub_attr.stSyncInfo.u16Vact == 0)) {
		vo_pub_attr.stSyncInfo.u16Hact = vo_width;
		vo_pub_attr.stSyncInfo.u16Vact = vo_height;
	}

	vo_layer_attr.stDispRect.s32X = (layer_width - vo_width) / 2;
	vo_layer_attr.stDispRect.s32Y = (layer_height - vo_height) / 2;
	vo_layer_attr.stDispRect.u32Width = vo_width;
	vo_layer_attr.stDispRect.u32Height = vo_height;
	vo_layer_attr.stImageSize.u32Width = vo_width;
	vo_layer_attr.stImageSize.u32Height = vo_height;
	RK_LOGI("vo_layer_attr W=%d, H=%d\n", vo_layer_attr.stDispRect.u32Width,
			vo_layer_attr.stDispRect.u32Height);

	vo_layer_attr.u32DispFrmRt = 30;
	vo_layer_attr.enPixFormat = RK_FMT_YUV420SP;
	vo_csc.enCscMatrix = VO_CSC_MATRIX_IDENTITY;
	vo_csc.u32Contrast = 50;
	vo_csc.u32Hue = 50;
	vo_csc.u32Luma = 50;
	vo_csc.u32Satuature = 50;

	/*bind layer0 to device hd0*/
	ret = RK_MPI_VO_BindLayer(vo_layer, vo_dev_id, VO_LAYER_MODE_GRAPHIC);
	if (ret != RK_SUCCESS) {
		RK_LOGE("failed to bind layer %#X\n", ret);
		return ret;
	}
	ret = RK_MPI_VO_SetLayerAttr(vo_layer, &vo_layer_attr);
	if (ret != RK_SUCCESS) {
		RK_LOGE("failed to set layer attr\n");
		return ret;
	}
	ret = RK_MPI_VO_SetLayerSpliceMode(vo_layer, VO_SPLICE_MODE_RGA);
	if (ret != RK_SUCCESS) {
		RK_LOGE("failed to set layer splice mode %#X\n", ret);
		return ret;
	}
	ret = RK_MPI_VO_EnableLayer(vo_layer);
	if (ret != RK_SUCCESS) {
		RK_LOGE("failed to enable layer %#X\n", ret);
		return ret;
	}
	ret = RK_MPI_VO_SetLayerCSC(vo_layer, &vo_csc);
	vo_chn_attr.bDeflicker = RK_FALSE;
	vo_chn_attr.u32Priority = vo_chn_priority;
	vo_chn_attr.stRect.s32X = 0;
	vo_chn_attr.stRect.s32Y = 0;
	vo_chn_attr.stRect.u32Width = vo_width;
	vo_chn_attr.stRect.u32Height = vo_height;
	vo_chn_attr.enRotation = vo_rotation;
	ret = RK_MPI_VO_SetChnAttr(vo_layer, vo_chn_id, &vo_chn_attr);
	ret = RK_MPI_VO_EnableChn(vo_layer, vo_chn_id);
	if (ret != RK_SUCCESS) {
		RK_LOGE("create %d layer %d ch vo failed!\n", vo_layer, vo_chn_id);
		return ret;
	}
	RK_MPI_VO_SetLayerCleanMode(vo_layer, VO_LAYER_CLEAN_MODE_ONCE);
	RK_LOGI("vo init success\n");
	return ret;
}

static int vo_deinit(void) {
	int ret = 0;
	int vo_layer = 0;
	int vo_dev_id = 0;
	int vo_chn_id = 0;
	if (rk_cmd_param.type != TYPE_GDC_VO)
		return 0;
	ret = RK_MPI_VO_DisableChn(vo_layer, vo_chn_id);
	if (ret)
		RK_LOGE("RK_MPI_VO_DisableChn failed, ret is %#x\n", ret);
	ret = RK_MPI_VO_DisableLayer(vo_layer);
	if (ret)
		RK_LOGE("RK_MPI_VO_DisableLayer failed, ret is %#x\n", ret);
	ret = RK_MPI_VO_Disable(vo_dev_id);
	if (ret)
		RK_LOGE("RK_MPI_VO_Disable failed, ret is %#x\n", ret);
	ret = RK_MPI_VO_UnBindLayer(vo_layer, vo_dev_id);
	if (ret)
		RK_LOGE("RK_MPI_VO_UnBindLayer failed, ret is %#x\n", ret);
	if (ret == RK_SUCCESS)
		RK_LOGI("vo deinit success\n");
	return ret;
}

static int gdc_init(void) {
	int ret = 0;
	int input_width, input_height, output_width, output_height;
	GDC_CHN_ATTR_S gdc_attr = {0};
	if (rk_cmd_param.type == TYPE_GDC_VO) {
		input_width = output_width = rk_cmd_param.display_width;
		input_height = output_height = rk_cmd_param.display_height;
	} else {
		input_width = rk_cmd_param.input_width;
		input_height = rk_cmd_param.input_height;
		output_width = rk_cmd_param.output_width;
		output_height = rk_cmd_param.output_height;
	}
	gdc_attr.u32MaxInQueue = 11;
	gdc_attr.u32MaxOutQueue = 4;
	gdc_attr.s32DstWidth = output_width;
	gdc_attr.s32DstHeight = output_height;
	gdc_attr.stEisAttr.s32DevNo = rk_cmd_param.iio_device_idx;
	if (rk_cmd_param.enable_compress && rk_cmd_param.type != TYPE_GDC_VO) {
		gdc_attr.enDstCompMode = COMPRESS_RFBC_64x4;
		gdc_attr.enDstPixelFormat = RK_FMT_YUV420SP;
	} else {
		gdc_attr.enDstCompMode = COMPRESS_MODE_NONE;
		gdc_attr.enDstPixelFormat = RK_FMT_YUV420SP;
	}
	if (rk_cmd_param.type == TYPE_GDC_GET_FRAME)
		gdc_attr.s32Depth = 1;
	memset(gdc_attr.cfgFile, 0, sizeof(gdc_attr.cfgFile));
	snprintf(gdc_attr.cfgFile, sizeof(gdc_attr.cfgFile)
			, "/oem/usr/share/rkeis_config/%s_normal_%dx%d_%dx%d_30fps.json"
			, rk_cmd_param.sensor_name
			, input_width, input_height, output_width
			, output_height);
	ret = RK_MPI_GDC_CreateChn(GDC_MAIN_CHN, &gdc_attr);
	if (ret) {
		RK_LOGE("ERROR: create GDC display channel error! ret=%#X\n", ret);
		return ret;
	}
	RK_LOGI("gdc init success, width %d, height %d", output_width, output_height);

	if (rk_cmd_param.enable_amtd || rk_cmd_param.enable_memc) {
		gdc_attr.stEisAttr.s32DevNo = rk_cmd_param.iio_device_idx;
		gdc_attr.u32MaxInQueue = 1;
		gdc_attr.u32MaxOutQueue = 1;
		gdc_attr.s32DstWidth = 480; // unused
		gdc_attr.s32DstHeight = 320; // unused
		gdc_attr.enDstPixelFormat = RK_FMT_YUV420SP;
		gdc_attr.enDstCompMode = COMPRESS_MODE_NONE;
		memset(gdc_attr.cfgFile, 0, sizeof(gdc_attr.cfgFile));
		GDC_INFO_CB_S gdc_info_cb = {};
		gdc_info_cb.pfnSensorCB = gdc_aiq_imu_callback;
		gdc_info_cb.pfnVframeCB = gdc_aiq_frame_callback;
		RK_MPI_GDC_Register_InfoCB(GDC_DUMMY_CHN, NULL, &gdc_info_cb);
		ret = RK_MPI_GDC_CreateChn(GDC_DUMMY_CHN, &gdc_attr);
		if (ret) {
			RK_LOGE("ERROR: create GDC dummy channel error! ret=%#X\n", ret);
			return ret;
		}
		RK_LOGI("gdc dummy init success");
	}
	// INFO: Wait for IMU data ready, do it in gdc is better?
	usleep(100 * 1000);
	return ret;
}

static int gdc_deinit(void) {
	int ret = 0;
	ret = RK_MPI_GDC_DestroyChn(GDC_MAIN_CHN);
	if (ret)
		RK_LOGE("ERROR: destroy GDC main channel error! ret=%#X\n", ret);
	if (rk_cmd_param.enable_amtd || rk_cmd_param.enable_memc) {
		ret = RK_MPI_GDC_DestroyChn(GDC_DUMMY_CHN);
		if (ret)
			RK_LOGE("ERROR: destroy GDC dummy channel error! ret=%#X\n", ret);
	}
	return ret;
}

static int venc_init(void) {
	VENC_CHN_ATTR_S venc_chn_attr;
	int ret = 0;
	int video_width = rk_cmd_param.output_width;
	int video_height = rk_cmd_param.output_height;
	memset(&venc_chn_attr, 0, sizeof(venc_chn_attr));
	if (rk_cmd_param.type != TYPE_GDC_VENC)
		return 0;
	venc_chn_attr.stVencAttr.enType = RK_VIDEO_ID_HEVC;
	venc_chn_attr.stRcAttr.enRcMode = VENC_RC_MODE_H265VBR;
	venc_chn_attr.stRcAttr.stH265Vbr.u32Gop = 30;
	venc_chn_attr.stRcAttr.stH265Vbr.u32BitRate = 10000;
	venc_chn_attr.stRcAttr.stH265Vbr.fr32DstFrameRateDen = 1;
	venc_chn_attr.stRcAttr.stH265Vbr.fr32DstFrameRateNum = rk_cmd_param.fps;
	venc_chn_attr.stRcAttr.stH265Vbr.u32SrcFrameRateDen = 1;
	venc_chn_attr.stRcAttr.stH265Vbr.u32SrcFrameRateNum = rk_cmd_param.fps;

	venc_chn_attr.stGopAttr.enGopMode = VENC_GOPMODE_NORMALP;
	// venc_chn_attr.stGopAttr.u32GopSize = rk_param_get_int("video.0:gop", -1);

	venc_chn_attr.stVencAttr.enPixelFormat = RK_FMT_YUV420SP;
	venc_chn_attr.stVencAttr.u32PicWidth = video_width;
	venc_chn_attr.stVencAttr.u32PicHeight = video_height;
	venc_chn_attr.stVencAttr.u32VirWidth = video_width;
	venc_chn_attr.stVencAttr.u32VirHeight = video_height;
	venc_chn_attr.stVencAttr.u32StreamBufCnt = 5;
	venc_chn_attr.stVencAttr.u32BufSize = video_width * video_height * 3;
	// venc_chn_attr.stVencAttr.u32Depth = 1;
	ret = RK_MPI_VENC_CreateChn(VENC_MAIN_CHN, &venc_chn_attr);
	if (ret) {
		RK_LOGE("ERROR: create VENC error! ret=%d\n", ret);
		return -1;
	}

	VENC_RC_PARAM_S venc_rc_param;
	RK_MPI_VENC_GetRcParam(VENC_MAIN_CHN, &venc_rc_param);
	venc_rc_param.s32FirstFrameStartQp = 26;
	venc_rc_param.stParamH265.u32StepQp = 4;
	venc_rc_param.stParamH265.u32MinQp = 10;
	venc_rc_param.stParamH265.u32MaxQp = 51;
	venc_rc_param.stParamH265.u32MinIQp = 10;
	venc_rc_param.stParamH265.u32MaxIQp = 51;
	venc_rc_param.stParamH265.s32DeltIpQp = -2;
	venc_rc_param.stParamH265.s32MaxReEncodeTimes = 2;
	venc_rc_param.stParamH265.u32FrmMinQp = 10;
	venc_rc_param.stParamH265.u32FrmMaxQp = 51;
	venc_rc_param.stParamH265.u32FrmMinIQp = 10;
	venc_rc_param.stParamH265.u32FrmMaxIQp = 51;
	RK_MPI_VENC_SetRcParam(VENC_MAIN_CHN, &venc_rc_param);

	VENC_RECV_PIC_PARAM_S recv_param;
	memset(&recv_param, 0, sizeof(VENC_RECV_PIC_PARAM_S));
	recv_param.s32RecvPicNum = -1;
	ret = RK_MPI_VENC_StartRecvFrame(VENC_MAIN_CHN, &recv_param);
	if (ret)
		RK_LOGE("RK_MPI_VENC_StartRecvFrame failed %#X", ret);
	RK_LOGI("venc init success\n");
	return ret;
}

static int venc_deinit(void) {
	int ret = 0;
	if (rk_cmd_param.type != TYPE_GDC_VENC)
		return 0;
	ret = RK_MPI_VENC_StopRecvFrame(VENC_MAIN_CHN);
	if (ret)
		RK_LOGE("ERROR: stop recv frame error! ret=%d\n", ret);
	ret = RK_MPI_VENC_DestroyChn(VENC_MAIN_CHN);
	if (ret) {
		RK_LOGE("ERROR: destroy VENC error! ret=%d\n", ret);
		return ret;
	}
	RK_LOGE("venc deinit success\n");
	return ret;
}

static int bind_pipe(void) {
	MPP_CHN_S vi_chn;
	MPP_CHN_S vo_chn;
	MPP_CHN_S gdc_chn;
	MPP_CHN_S venc_chn;
	int ret = 0;
	int vi_main_chn = rk_cmd_param.enable_vpss ? VPSS_ONLINE_MAIN_CHN : ISP_ONLINE_MAIN_CHN;
	vi_chn.enModId = RK_ID_VI;
	vi_chn.s32DevId = VI_DEV;
	vi_chn.s32ChnId = vi_main_chn;
	vo_chn.enModId = RK_ID_VO;
	vo_chn.s32DevId = 0;
	vo_chn.s32ChnId = 0;
	gdc_chn.enModId = RK_ID_GDC;
	gdc_chn.s32DevId = 0;
	gdc_chn.s32ChnId = GDC_MAIN_CHN;
	venc_chn.enModId = RK_ID_VENC;
	venc_chn.s32DevId = 0;
	venc_chn.s32ChnId = VENC_MAIN_CHN;

	ret = RK_MPI_SYS_Bind(&vi_chn, &gdc_chn);
	if (ret) {
		RK_LOGE("ERROR: bind vi to gdc error! ret=%#X\n", ret);
		return ret;
	}
	if (rk_cmd_param.type == TYPE_GDC_VO) {
		ret = RK_MPI_SYS_Bind(&gdc_chn, &vo_chn);
		if (ret) {
			RK_LOGE("ERROR: bind gdc to vo error! ret=%#X\n", ret);
			return ret;
		}
	} else if (rk_cmd_param.type == TYPE_GDC_VENC) {
		ret = RK_MPI_SYS_Bind(&gdc_chn, &venc_chn);
		if (ret) {
			RK_LOGE("ERROR: bind gdc to venc error! ret=%#X\n", ret);
			return ret;
		}
	}
	return ret;
}

static int unbind_pipe(void) {
	int ret = 0;
	int vi_main_chn = rk_cmd_param.enable_vpss ? VPSS_ONLINE_MAIN_CHN : ISP_ONLINE_MAIN_CHN;
	MPP_CHN_S vi_chn;
	MPP_CHN_S vo_chn;
	MPP_CHN_S gdc_chn;
	MPP_CHN_S venc_chn;

	vi_chn.enModId = RK_ID_VI;
	vi_chn.s32DevId = VI_DEV;
	vi_chn.s32ChnId = vi_main_chn;
	vo_chn.enModId = RK_ID_VO;
	vo_chn.s32DevId = 0;
	vo_chn.s32ChnId = 0;
	gdc_chn.enModId = RK_ID_GDC;
	gdc_chn.s32DevId = 0;
	gdc_chn.s32ChnId = GDC_MAIN_CHN;
	venc_chn.enModId = RK_ID_VENC;
	venc_chn.s32DevId = 0;
	venc_chn.s32ChnId = VENC_MAIN_CHN;

	if (rk_cmd_param.type == TYPE_GDC_VO) {
		ret = RK_MPI_SYS_UnBind(&gdc_chn, &vo_chn);
		if (ret) {
			RK_LOGE("ERROR: unbind gdc to vo error! ret=%#X\n", ret);
			return ret;
		}
	} else if (rk_cmd_param.type == TYPE_GDC_VENC) {
		ret = RK_MPI_SYS_UnBind(&gdc_chn, &venc_chn);
		if (ret) {
			RK_LOGE("ERROR: unbind gdc to venc error! ret=%#X\n", ret);
			return ret;
		}
	}
	ret = RK_MPI_SYS_UnBind(&vi_chn, &gdc_chn);
	if (ret) {
		RK_LOGE("ERROR: unbind vi to gdc error! ret=%#X\n", ret);
		return ret;
	}
	return ret;
}

int main(int argc, const char **argv) {
	int ret = 0;
	VENC_STREAM_S video_stream = {};
	video_stream.pstPack = malloc(sizeof(VENC_PACK_S));
	signal(SIGINT, sigterm_handler);
	signal(SIGTERM, sigterm_handler);
	ret = parse_cmd_args(argc, argv);
	if (ret)
		return ret;
	RK_MPI_SYS_Init();
	system("cat /proc/meminfo > /tmp/eis_before_meminfo.txt");
	system("cat /sys/kernel/debug/dma_buf/bufinfo >> /tmp/eis_before_meminfo.txt");
	system("cat /proc/vmallocinfo >> /tmp/eis_before_meminfo.txt");
	for (int loop = 0; loop < rk_cmd_param.loop_count && !quit; loop++) {
		if (rk_cmd_param.enable_test)
			rk_cmd_param.type = (loop % TYPE_MAX_NUM);
		ret = isp_init();
		if (ret) {
			RK_LOGE("isp init failed\n");
			break;
		}
		ret = vi_dev_init();
		if (ret) {
			RK_LOGE("vi dev init failed\n");
			break;
		}
		ret = vi_chn_init();
		if (ret) {
			RK_LOGE("vi chn init failed\n");
			break;
		}
		ret = gdc_init();
		if (ret) {
			RK_LOGE("gdc init failed\n");
			break;
		}
		ret = vo_init();
		if (ret) {
			RK_LOGE("vo init failed\n");
			break;
		}
		ret = venc_init();
		if (ret) {
			RK_LOGE("venc init failed\n");
			break;
		}
		ret = bind_pipe();
		if (ret) {
			RK_LOGE("bind pipe failed\n");
			break;
		}
		if (rk_cmd_param.type == TYPE_GDC_VO) {
			int i = 0;
			while (!quit) {
				usleep(33 * 1000);
				if (rk_cmd_param.frame_count > 0 && ++i > rk_cmd_param.frame_count)
					break;
			}
		} else if (rk_cmd_param.type == TYPE_GDC_VENC) {
			int i = 0;
			__retry_get_venc_frame:
			while (!quit) {
				ret = RK_MPI_VENC_GetStream(VENC_MAIN_CHN, &video_stream, 1000);
				if (ret) {
					RK_LOGE("ERROR: get venc stream error! ret=%#X\n", ret);
					goto __retry_get_venc_frame;
				} else {
					RK_LOGI("get venc stream %d, pts %lld\n", i,
							video_stream.pstPack->u64PTS);
					RK_MPI_VENC_ReleaseStream(VENC_MAIN_CHN, &video_stream);
				}
				if (rk_cmd_param.frame_count > 0 && ++i > rk_cmd_param.frame_count)
					break;
			}
		} else if (rk_cmd_param.type == TYPE_GDC_GET_FRAME) {
			int i = 0;
			__retry_get_gdc_frame:
			while (!quit) {
				VIDEO_FRAME_INFO_S gdc_frame;
				ret = RK_MPI_GDC_GetFrame(GDC_MAIN_CHN, &gdc_frame, 1000);
				if (ret) {
					RK_LOGE("ERROR: get gdc frame error! ret=%#X\n", ret);
					goto __retry_get_gdc_frame;
				} else {
					RK_LOGI("get gdc frame %d, pts %lld\n", i, gdc_frame.stVFrame.u64PTS);
					RK_MPI_GDC_ReleaseFrame(GDC_MAIN_CHN, &gdc_frame);
				}
				if (rk_cmd_param.frame_count > 0 && ++i > rk_cmd_param.frame_count)
					break;
			}
		}
		ret = unbind_pipe();
		if (ret) {
			RK_LOGE("unbind pipe failed\n");
			break;
		}
		ret = venc_deinit();
		if (ret) {
			RK_LOGE("venc deinit failed\n");
			break;
		}
		ret = vo_deinit();
		if (ret) {
			RK_LOGE("vo deinit failed\n");
			break;
		}
		ret = gdc_deinit();
		if (ret) {
			RK_LOGE("gdc deinit failed\n");
			break;
		}
		ret = vi_chn_deinit();
		if (ret) {
			RK_LOGE("vi chn deinit failed\n");
			break;
		}
		ret = vi_dev_deinit();
		if (ret) {
			RK_LOGE("vi dev deinit failed\n");
			break;
		}
		ret = isp_deinit();
		if (ret) {
			RK_LOGE("isp init failed\n");
			break;
		}
		RK_LOGE(
			"=========================== loop %d success ============================\n",
			loop);
	}
	system("cat /proc/meminfo > /tmp/eis_after_meminfo.txt");
	system("cat /sys/kernel/debug/dma_buf/bufinfo >> /tmp/eis_after_meminfo.txt");
	system("cat /proc/vmallocinfo >> /tmp/eis_after_meminfo.txt");
	fprintf(stderr, "%s: exit stress test\n", __func__);
	RK_MPI_SYS_Exit();
	if (video_stream.pstPack)
		free(video_stream.pstPack);
	return ret;
}
