/***********************************************************************************************************************
*                                 Copyright (c) 2018,深圳创成微电子<-All rights reserved->
*               
**----------------------------------------------------------------------------------------------------------------------
*   文件名称：   enc_core.h
*   简要描述:    ENC 算法帧处理核心头文件
*
**----------------------------------------------------------------------------------------------------------------------
*   当前版本：   V1.0
*   作者/修改者：王自振
*   完成日期：   2018年11月08日
**----------------------------------------------------------------------------------------------------------------------
*   取代版本：
*   原作者  ：   
*   完成日期：
***********************************************************************************************************************/
#ifndef __ENC_CORE_H__
#define __ENC_CORE_H__

#define USER_EXP_LOG                                             /* 是否使用自定义log exp 函数    */
//#define USE_HARDWARE_FFT                                         /* 是否使用硬件FFT              */

#include "stdint.h"
#include "window.h"
#include "ns_fft.h"

/***********************************************************************************************************************
** 重要ENC全局结构体存储位置(需要16kBytes 空间)
***********************************************************************************************************************/
/***********************************************************************************************************************
** 选择使用math库中的exp log 还是使用自定义的exp log
***********************************************************************************************************************/

#ifdef USER_EXP_LOG                                                 /* 使用自定义的log 和 exp函数    */
	#define  log_opt  user_log                                      
	#define  exp_opt  user_exp
    #define LN_10  2.30258509299404568401f
	#define  _SPL_FLOAT_MIN -3.4028235E38f
	#define  _SPL_FLOAT_MAX  3.4028235E38f
#else                                                               /* 使用数学库的log 和 exp函数    */
	#define  log_opt  log
	#define  exp_opt  exp
#endif

/***********************************************************************************************************************
** 常量宏声明
***********************************************************************************************************************/
#define ANAL_BLOCKL_MAX     256                                      /* FFT分析点数 */
#define HALF_ANAL_BLOCKL    129                                      /* 实际频谱点数 */

#define QUANTILE            (float)0.5f
#define FACTOR              (float)40.0f
#define WIDTH               (float)0.01f
#define END_STARTUP_LONG    200

/* 一组初始调节参数 */
#define DD_PR_SNR           (float)0.98f 
#define LRT_TAVG            (float)0.50f 
#define SPECT_FL_TAVG       (float)0.30f  
#define SPECT_DIFF_TAVG     (float)0.70f 
#define PRIOR_UPDATE        (float)0.80f 
#define NOISE_UPDATE        (float)0.60f 
#define SPEECH_UPDATE       (float)0.99f 
#define WIDTH_PR_MAP        (float)4.0f  
#define LRT_FEATURE_THR     (float)0.5f  
#define SF_FEATURE_THR      (float)0.5f 
#define SD_FEATURE_THR      (float)0.5f 
#define PROB_RANGE          (float)0.60f 

#define HIST_PAR_EST         1000                                    /* 直方图size */
#define GAMMA_PAUSE         (float)0.05f 

#define  _SPL_WORD16_MAX        32767
#define  _SPL_WORD16_MIN       -32768

/***********************************************************************************************************************
** 数据结构体声明
***********************************************************************************************************************/
typedef struct NSParaExtract_t_ {
	float binSizeSpecDiff;                              /* 直方图的每个点的size                */
	float factor1ModelPars;                             /* 给似然比和频谱差异用的缩放参数       */
	float limitPeakSpacingSpecDiff;                     /* 直方图中两个最高峰之间的空白的限制    */
	float limitPeakWeightsSpecDiff;                     /* 两个峰值之间的相关性的限制值         */
	float maxSpecDiff;                                  /* 频谱差异最大值                      */
	float minSpecDiff;                                  /* 频谱差异最小值                      */
} NSParaExtract_t;

typedef  struct Spec_t_{
    float  magn[HALF_ANAL_BLOCKL];
    float noise[HALF_ANAL_BLOCKL];
}Spec_t;

/************ 时域数据和FFT输出共用体***************/
typedef union  Tmp_Buf_t_ {
	float winData[2*HALF_ANAL_BLOCKL];
    Spec_t spec;
}  Tmp_Buf_t;


/************ENC 算法数据结构体 ********************/
typedef struct NSinst_t_ {

	nscore_fft_instance fft_inst;
	nscore_fft_instance ifft_inst;

	float                              overdrive;       /* 控制过估计的参数约等1   [ 常量可使用宏声明替代 ]            */
	float                           denoiseBound;       /* 维纳滤波系数下限       [ 常量可使用宏声明替代 ]             */

	unsigned short                            fs;       /* 音频采样率             [ 常量可使用宏声明替代 ]             */
	unsigned short                      frameLen;       /* 帧步进大小             [ 常量可使用宏声明替代 ]             */
    unsigned short                          nfft;       /* FFT 分析帧大小         [ 常量可使用宏声明替代 ]             */
    unsigned short                       magnLen;       /* 扫描频点数 = nfft/2+1  [ 常量可使用宏声明替代 ]             */

    float                           signalEnergy;       /* 频谱模值平方和 [ 无帧外迭代，可用局部变量，有传参需求故用全局] */
	float                                sumMagn;       /* 频谱模值和     [ 无帧外迭代，可用局部变量，有传参需求故用全局] */

	/********* 数据输入输出 *****************/
	const float*                          window;       /* 加窗数据指针               */
	float                               blockInd;       /* 帧计数器                   */
	short             dataBufLB[ANAL_BLOCKL_MAX];       /* 低频输入空间               */
	float               syntBuf[ANAL_BLOCKL_MAX];       /* 低频重叠拼接空间           */

    /********* 分位数噪声估计相关 **********/
    float           priorSpeechProb;                     /* 语音和噪声的先验概率      */
   	int                    counter0;                     /* 分位数更新状态机计数      */
	float             density[ HALF_ANAL_BLOCKL ];       /* 概率密度存储空间          */
	float           lquantile[ HALF_ANAL_BLOCKL ];       /* 对数分位数存储空间        */
	float            quantile[ HALF_ANAL_BLOCKL ];       /* 线性分位数存储空间        */
	float              smooth[ HALF_ANAL_BLOCKL ];       /* 维纳频域滤波器系数        */

    /*********  频谱差异比较计算 *********/
    float            magnPrev[ HALF_ANAL_BLOCKL ];       /* 上一帧的幅度谱             */
	float           noisePrev[ HALF_ANAL_BLOCKL ];       /* 上一帧的噪声谱             */
	float       logLrtTimeAvg[ HALF_ANAL_BLOCKL ];       /* 平滑的似然比，其实就是门限 */
    float        magnAvgPause[ HALF_ANAL_BLOCKL ];       /* 保守噪声谱估计             */
	int                histSpecDiff[HIST_PAR_EST];       /* 直方图存储空间             */
	NSParaExtract_t featureExtractionParams;             /* 频谱差异结构体指针         */

    /*	直方图更新状态机        */
	int modelUpdateMode ;                                /* 更新标志 0不更新，1 更新一次 2 每窗更新 */
    int modelUpdateFrame ;                               /* 更新窗口                                */
    int modelUpdateCnt ;                                 /* 状态机计数器                            */
	float priorModelPars3;                     
	       
	float featureData4;
	float featureData5;
	float featureData6;

	/* 大块内存不会在帧外迭代， 但是作为临时变量会造成堆栈溢出故用全局 */
    Tmp_Buf_t                                 Tbuf;       /* fft时域数据，时分复用幅度谱magn 噪声谱 noise */
    float                 fftData[ANAL_BLOCKL_MAX];       /* fft频域数据  */
    float             snrLocPost[HALF_ANAL_BLOCKL];
    float            snrLocPrior[HALF_ANAL_BLOCKL];
    float        probSpeechFinal[HALF_ANAL_BLOCKL];
    float   previousEstimateStsa[HALF_ANAL_BLOCKL];

} NSinst_t;

#endif // __ENC_CORE_H__
