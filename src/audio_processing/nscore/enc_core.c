/***********************************************************************************************************************
*                                 Copyright (c) 2018,深圳创成微电子<-All rights reserved->
*               
**----------------------------------------------------------------------------------------------------------------------
*   文件名称：   enc_core.c
*   简要描述:    环境降噪帧处理内核
*
**----------------------------------------------------------------------------------------------------------------------
*   当前版本：   V1.0
*   作者/修改者：王自振
*   完成日期：   2021年11月18日
**----------------------------------------------------------------------------------------------------------------------
*   取代版本： 
*   原作者  ： 
*   完成日期： 
***********************************************************************************************************************/
#include <math.h>
#include <string.h>
#include "enc_core.h"
#include "nscore.h"

/* ESP32-P4 port: 16-kHz mono LB, portable FFT, no STM32 timing/HB path. */
static NSinst_t inst;
static float ola_normalizer[160];

static void Ns_InitCore( void)
{
    int i;
      
    inst.fs       = 16000;             // 采样率
    inst.frameLen = 160;               // 帧长度
    inst.nfft     = 256;               // 分析窗长度 就是FFT点数
    inst.window   = window256;         // 混合汉宁平顶窗数据指针
    inst.magnLen  = inst.nfft / 2 + 1; //频点的个数  
    nscore_fft_init(&( inst.fft_inst), inst.nfft);
    nscore_fft_init(&(inst.ifft_inst), inst.nfft);


    for (i = 0; i < ANAL_BLOCKL_MAX; i++) {
        /* 输入输出空间初始化 */
        inst.dataBufLB[i]     = 0x0;
        inst.syntBuf[i]       = (float)0.0f;
    }

    for (i = 0; i < HALF_ANAL_BLOCKL; i++) {
        /* 分位数噪声估计相关 */
        inst.density[i]       = (float)0.3f;
        inst.lquantile[i]     = (float)8.0f;
        inst.quantile[i]      = (float)0.0f;
        inst.smooth[i]        = (float)1.0f;

        /* 频谱差异比较相关 */
        inst.magnPrev[i]      = (float)0.0f;
        inst.noisePrev[i]     = (float)0.0f; 
        inst.logLrtTimeAvg[i] = LRT_FEATURE_THR; 
        inst.magnAvgPause[i]  = (float)0.0f;  
    }

    for (i = 0; i < HIST_PAR_EST; i++) {
        /* 直方图数据初始化 */
        inst.histSpecDiff[i]  = 0;
    }

    inst.counter0        =  END_STARTUP_LONG;
    inst.priorSpeechProb   = (float)0.5f; //语音和噪声的先验概率

    inst.featureData4    = SF_FEATURE_THR;  
    inst.featureData5    = (float)0.0f;      
    inst.featureData6    = (float)0.0f;      

    inst.blockInd           = -1;
    inst.priorModelPars3  = (float)0.5f;      //模板差异的门限值

    inst.modelUpdateMode = 2;                  //更新标志 0不更新，1 更新一次 2 每窗更新
    inst.modelUpdateFrame = 500;               //更新窗口    
    inst.modelUpdateCnt = inst.modelUpdateFrame;//

    inst.signalEnergy       = 0.0f;
    inst.sumMagn            = 0.0f;

     /*  设置频谱差异特征提取使用的参数 */
    inst.featureExtractionParams.binSizeSpecDiff          = (float)0.1f;
    inst.featureExtractionParams.factor1ModelPars         = (float)1.20f; 
    inst.featureExtractionParams.limitPeakSpacingSpecDiff = (float)0.2f;
    inst.featureExtractionParams.limitPeakWeightsSpecDiff = (float)0.5f;
    inst.featureExtractionParams.maxSpecDiff              = (float)1.0f;
    inst.featureExtractionParams.minSpecDiff              = (float)0.16f;
 
    /* 设置降噪策略参数，用户控制接口    */
    inst.overdrive = (float)1.0f;
    inst.denoiseBound = (float)0.25f;
}

/**********************************************************************************************************************
*   函数名称： user_exp
*   功能描述： 近似的指数实现
*   参数列表： exp_in : 输入数据
*   硬件输出： 无
*   返回结果： 无
*   历史记录： 作者/修改者            日期                    备注
*              wzz                 2021.11.18               原始创建版本V1.0
*
**********************************************************************************************************************/
#ifdef USER_EXP_LOG
static float exp_opt(float exp_in) 
{
    float exp_temp;

    exp_temp = exp_in;
    exp_temp = 1.0f + exp_temp / 1024;
    exp_temp *= exp_temp; exp_temp *= exp_temp; exp_temp *= exp_temp; exp_temp *= exp_temp;
    exp_temp *= exp_temp; exp_temp *= exp_temp; exp_temp *= exp_temp; exp_temp *= exp_temp;
    exp_temp *= exp_temp; exp_temp *= exp_temp;

    if (exp_temp <  _SPL_FLOAT_MIN) {
        exp_temp =  _SPL_FLOAT_MIN;
    }
    else if (exp_temp >  _SPL_FLOAT_MAX) {
        exp_temp =  _SPL_FLOAT_MAX;
    }

    return exp_temp;
}
#endif

/**********************************************************************************************************************
*   函数名称： user_log
*   功能描述： 近似的对数实现
*   参数列表： log_in : 输入数据
*   硬件输出： 无
*   返回结果： 无
*   历史记录： 作者/修改者            日期                    备注
*              wzz                 2021.11.18               原始创建版本V1.0
*
**********************************************************************************************************************/
#ifdef USER_EXP_LOG
static float log_opt(float log_in) 
{
    int count_log;
    float log_1, log_2, log_3;
    float tmpFloat1;

    tmpFloat1 = log_in;
    count_log = 0;
    if (tmpFloat1 > 10.0f) {
        do {
            tmpFloat1 = tmpFloat1 / 10.0f;
            count_log++;
        } while (tmpFloat1 > 10.0f);

        tmpFloat1 = (tmpFloat1 - 1.0f) / (tmpFloat1 + 1.0f);
        log_1 = (1.0f / 3)*(tmpFloat1 * tmpFloat1 * tmpFloat1);
        log_2 = (1.0f / 5)*(tmpFloat1 * tmpFloat1 * tmpFloat1 * tmpFloat1 * tmpFloat1);
        log_3 = (1.0f / 7)*(tmpFloat1 * tmpFloat1 * tmpFloat1 * tmpFloat1 * tmpFloat1 * tmpFloat1 * tmpFloat1);
        tmpFloat1 = 2.0f * (tmpFloat1 + log_1 + log_2 + log_3);

        tmpFloat1 = (float)(tmpFloat1 + count_log * LN_10);
    }
    else {
        tmpFloat1 = (tmpFloat1 - 1.0f) / (tmpFloat1 + 1.0f);
        log_1 = (1.0f / 3)*(tmpFloat1 * tmpFloat1 * tmpFloat1);
        log_2 = (1.0f / 5)*(tmpFloat1 * tmpFloat1 * tmpFloat1 * tmpFloat1 * tmpFloat1);
        log_3 = (1.0f / 7)*(tmpFloat1 * tmpFloat1 * tmpFloat1 * tmpFloat1 * tmpFloat1 * tmpFloat1 * tmpFloat1);
        tmpFloat1 = 2.0f * (tmpFloat1 + log_1 + log_2 + log_3);
    }

    return tmpFloat1;
}
#endif


/**********************************************************************************************************************
*   函数名称： Ns_NoiseEstimation
*   功能描述： 分位数噪声估计
*   参数列表： magn：帧频谱 noise：噪声频谱
*   硬件输出： 无
*   返回结果： 无
*   历史记录： 作者/修改者            日期                    备注
*              wzz                 2021.11.18               原始创建版本V1.0
*
**********************************************************************************************************************/
static void  Ns_NoiseEstimation( float* magn, float* noise)
{
    int i ;
    float lmagn, delta;

    /* 每一帧 的 每一个频点 */
    for (i = 0; i < inst.magnLen; i++) {
        lmagn = (float)log_opt(magn[i]);                                                         

        if (inst.density[ i] > 1.0f) {                                                        
            delta = FACTOR * (float)1.0f / inst.density[ i];
        }
        else {
            delta = FACTOR;
        }

        /* 更新对数分位数 */
        if (lmagn > inst.lquantile[ i]) {
            inst.lquantile[ i] += QUANTILE * delta / (float)(inst.counter0 + 1);
        }
        else {
            inst.lquantile[ i] -= ((float)1.0f - QUANTILE) * delta/ (float)(inst.counter0 + 1);
        }

        /* 更新能量密度估计 */
        if (fabsf(lmagn - inst.lquantile[ i]) < WIDTH) {
            inst.density[ i] = ((float)inst.counter0 * inst.density[i] + 
                               (float)1.0f / ((float)2.0f * WIDTH))/ (float)(inst.counter0 + 1);
        }
    } // 频点循环

    if (inst.counter0 >= END_STARTUP_LONG) {      
        inst.counter0 = 0;    
        for (i = 0; i < inst.magnLen; i++) {
            inst.quantile[i] = (float)exp_opt(inst.lquantile[ i]);
        }
    }
    inst.counter0++;

    for (i = 0; i < inst.magnLen; i++) {
        noise[i] = inst.quantile[i];
    }
}


/**********************************************************************************************************************
*   函数名称： Ns_ComputeSpectralDifference
*   功能描述： 输入频谱与噪声频谱形状的偏差
*   参数列表： magnIn 新的帧频谱
*   硬件输出： 无
*   返回结果： 无
*   历史记录： 作者/修改者            日期                    备注
*              wzz                 2021.11.18               原始创建版本V1.0
*
**********************************************************************************************************************/
static void  Ns_ComputeSpectralDifference( float* magnIn) 
{
                                                                        //
    int i;
    float avgPause, avgMagn, covMagnPause, varPause, varMagn ;

    /*** 局部变量内存优化*******/
    #define avgDiffNormMagn  avgPause

    avgPause = 0.0f;
    avgMagn = inst.sumMagn;

    for (i = 0; i < inst.magnLen; i++) {
        avgPause += inst.magnAvgPause[i];
    }
    avgPause     = avgPause / ((float)inst.magnLen);                                          /* 噪声频谱均值                  */
    avgMagn      = avgMagn  / ((float)inst.magnLen);                                          /* 输入频谱均值                  */
 
  /* 求方差 和 协方差 */
    covMagnPause = 0.0f;
    varPause     = 0.0f;
    varMagn      = 0.0f;
    for (i = 0; i < inst.magnLen; i++) {
        covMagnPause += (magnIn[i] - avgMagn) * (inst.magnAvgPause[i] - avgPause);            /* 输入频谱 与 噪声频谱协方差 分子 */
        varPause += (inst.magnAvgPause[i] - avgPause) * (inst.magnAvgPause[i] - avgPause);    /* 噪声频谱方差 分子              */
        varMagn += (magnIn[i] - avgMagn) * (magnIn[i] - avgMagn)  ;                           /* 输入频谱方差： 分子            */
    }
    covMagnPause = covMagnPause / ((float)inst.magnLen-1);                                    /* 输入频谱与噪声频谱协方差 相关性 */
    varPause = varPause / ((float)inst.magnLen-1);                                            /* 噪声频谱方差                   */
    varMagn = varMagn / ((float)inst.magnLen-1);                                              /* 输入频谱方差                   */
                                               
    inst.featureData6 += inst.signalEnergy;

    avgDiffNormMagn = varMagn - (covMagnPause * covMagnPause) / (varPause + (float)0.0001f);   /*归一化 并 时间平均              */
    avgDiffNormMagn = (float)(avgDiffNormMagn / (inst.featureData5 + (float)0.0001f));
    inst.featureData4 += SPECT_DIFF_TAVG * (avgDiffNormMagn - inst.featureData4);
}


/**********************************************************************************************************************
*   函数名称： Ns_FeatureParameterExtraction
*   功能描述： 提取频谱差异度特征参数的阈值
*   参数列表： flag：是否更新直方图
*   硬件输出： 无
*   返回结果： 无
*   历史记录： 作者/修改者            日期                    备注
*              wzz                 2021.11.18               原始创建版本V1.0
*
**********************************************************************************************************************/
static void  Ns_FeatureParameterExtraction( int flag) 
{
    int i , maxPeak1, maxPeak2;
    int weightPeak1SpecDiff, weightPeak2SpecDiff;

    float binMid ;
    float posPeak1SpecDiff, posPeak2SpecDiff;


    /* 更新直方图 */
    if (flag == 0) {
        /* 频谱差异 */
        if ((inst.featureData4 < 
           HIST_PAR_EST* inst.featureExtractionParams.binSizeSpecDiff)
            && (inst.featureData4 >= 0.0f)) {
            i = (int)(inst.featureData4 / inst.featureExtractionParams.binSizeSpecDiff);
            inst.histSpecDiff[i]++;
        }
    }

    /*在计算语音噪声的概率 */
    if (flag == 1) {
        /*计算频谱差异的两个峰值 */
        maxPeak1            = 0;    /* 最大值 */
        maxPeak2            = 0;    /* 次大值 */
        weightPeak1SpecDiff = 0;
        weightPeak2SpecDiff = 0;
        posPeak1SpecDiff    = 0.0f;
        posPeak2SpecDiff    = 0.0f;
        /*频谱差异的峰值 */
        for (i = 0; i < HIST_PAR_EST; i++) {
            binMid = ((float)i + (float)0.5f) * inst.featureExtractionParams.binSizeSpecDiff;
            if (inst.histSpecDiff[i] > maxPeak1) {
                /*找到新的第一个峰值    */
                /* 最大值的属性给次大值 */
                maxPeak2            = maxPeak1;
                weightPeak2SpecDiff = weightPeak1SpecDiff;
                posPeak2SpecDiff    = posPeak1SpecDiff;
                /* 更新新的最大值属性 */
                maxPeak1            = inst.histSpecDiff[i];
                weightPeak1SpecDiff = inst.histSpecDiff[i];
                posPeak1SpecDiff    = binMid;
            }
            else if (inst.histSpecDiff[i] > maxPeak2) {
                /*找到新的第二个峰值 */
                maxPeak2            = inst.histSpecDiff[i];
                weightPeak2SpecDiff = inst.histSpecDiff[i];
                posPeak2SpecDiff    = binMid;
            }
        }

        /* 如果两个峰值太近，就合并 */
        if ((fabsf(posPeak2SpecDiff - posPeak1SpecDiff)< inst.featureExtractionParams.limitPeakSpacingSpecDiff)&& 
           (weightPeak2SpecDiff > weightPeak1SpecDiff * inst.featureExtractionParams.limitPeakWeightsSpecDiff)) {
            weightPeak1SpecDiff += weightPeak2SpecDiff;
            posPeak1SpecDiff = (float)0.5f * (posPeak1SpecDiff + posPeak2SpecDiff);
        }
        /* 算一下门限值 */
        inst.priorModelPars3 = inst.featureExtractionParams.factor1ModelPars* posPeak1SpecDiff;

        /* 检查下值是否在最大值和最小值之间 */
        if (inst.priorModelPars3 < inst.featureExtractionParams.minSpecDiff) {
            inst.priorModelPars3 = inst.featureExtractionParams.minSpecDiff;
        }
        if (inst.priorModelPars3 > inst.featureExtractionParams.maxSpecDiff) {
            inst.priorModelPars3 = inst.featureExtractionParams.maxSpecDiff;
        }

        // if (inst.modelUpdateMode >= 1) {
            for (i = 0; i < HIST_PAR_EST; i++) {
                inst.histSpecDiff[i] = 0;
            }
        // }
    } 
}

/**********************************************************************************************************************
*   函数名称： Ns_SpeechNoiseProb
*   功能描述： 计算最终信噪比信噪比
*   参数列表： 算法结构体
*   硬件输出： 无
*   返回结果： 无
*   历史记录： 作者/修改者            日期                    备注
*              wzz                 2021.11.18               原始创建版本V1.0
*
**********************************************************************************************************************/
static void  Ns_SpeechNoiseProb( float*    probSpeechFinal, 
                                 float*    snrLocPrior,
                                 float*    snrLocPost )
{
    (void)probSpeechFinal;
    (void)snrLocPrior;
    (void)snrLocPost;
    int i ;
    float tmpFloat1, tmpFloat2;

    /**** 局部变量内存优化 *******/
    #define indPrior    tmpFloat1
    #define widthPrior  tmpFloat2
    #define besselTmp   tmpFloat2
    #define invLrt      tmpFloat1
    #define gainPrior   tmpFloat2

    for (i = 0; i < inst.magnLen; i++) {
        tmpFloat1 = (float)1.0f + (float)2.0f * inst.snrLocPrior[i];
        tmpFloat2 = (float)2.0f * inst.snrLocPrior[i] / (tmpFloat1 + (float)0.0001f);
        besselTmp = (inst.snrLocPost[i] + (float)1.0f) * tmpFloat2;
        inst.logLrtTimeAvg[i] += LRT_TAVG * (besselTmp - (float)log_opt(tmpFloat1) - inst.logLrtTimeAvg[i]);       
    }

    if ( inst.priorModelPars3 > inst.featureData4 ) {
        widthPrior = (float)2.0f * WIDTH_PR_MAP;
    } else {
        widthPrior = WIDTH_PR_MAP;
    }

    tmpFloat1 = widthPrior * (inst.featureData4 - inst.priorModelPars3);
    tmpFloat2 = exp_opt(tmpFloat1);
    tmpFloat1 = tmpFloat2*tmpFloat2;
    tmpFloat2 = (float)(1.0f / tmpFloat1);

    indPrior  = (float)(1.0f /(1 + tmpFloat2));// 
    inst.priorSpeechProb += PRIOR_UPDATE * (indPrior - inst.priorSpeechProb);  /* 每一帧 更新语音/噪声概率 */
     
    /* 限定最大最小概率值 */                                                                    
    if (inst.priorSpeechProb > 1.0f)  { inst.priorSpeechProb  = (float)1.0f ; }
    if (inst.priorSpeechProb < 0.01f) { inst.priorSpeechProb  = (float)0.01f; }
    
    gainPrior = ((float)1.0f - inst.priorSpeechProb) / (inst.priorSpeechProb + (float)0.0001f); 
    for (i = 0; i < inst.magnLen; i++) {
        invLrt = (float)exp_opt(-inst.logLrtTimeAvg[i]);
        invLrt = (float)gainPrior * invLrt;
        inst.probSpeechFinal[i] = (float)1.0f / ((float)1.0f + invLrt);
    }
}

/**********************************************************************************************************************
*   函数名称： Ns_ProcessCore
*   功能描述： 固件主函数
*   参数列表： 无
*   硬件输出： 无
*   返回结果： 无
*   历史记录： 作者/修改者            日期                    备注
*              wzz                 2021.11.17               原始创建版本V1.0
*
**********************************************************************************************************************/
static int Ns_ProcessCore( short* inFrameLB, 
                     short* inFrameHB, 
                     short* outFrameLB, 
                     short* outFrameHB)
{
    int     i;
    (void)inFrameHB;
    (void)outFrameHB;

    /********* 局部变量************************/
    float tmpData0;
    float tmpData1;
    float tmpData2;
    float tmpData3;
    float tmpData4;
    /********* 局部变量内存优化 ***************/
    #define    energy1                tmpData0
    #define    snrPrior               tmpData0
    #define    currentEstimateStsa    tmpData1
    #define    tmpFloat1              tmpData2
    #define    probSpeech             tmpData0
    #define    probNonSpeech          tmpData1
    #define    gammaNoiseTmp          tmpData2
    #define    gammaNoiseOld          tmpData3
    #define    noiseUpdateTmp         tmpData4
    #define    fTmp                   tmpData0
    #define    dTmp                   tmpData1
    // #define    deltaBweHB             tmpData0

    /*************************************************************************************************************************
    * 准备输入数据
    *************************************************************************************************************************/
    /* 流方式填充低频FIFO */
    for (i = 0; i < inst.nfft - inst.frameLen; i++) {
        inst.dataBufLB[i] = inst.dataBufLB[inst.frameLen + i];
    }
    for (i = 0; i < inst.frameLen; i++) {
        inst.dataBufLB[inst.nfft - inst.frameLen + i] = inFrameLB[i]; // 
    }

    /*************************************************************************************************************************
    *输入数据加窗
    *************************************************************************************************************************/
    energy1 = 0.0f;
    for (i = 0; i < inst.nfft; i++) {
        inst.Tbuf.winData[i] = inst.window[i] * ((float)inst.dataBufLB[i]);                         /* 为低频数据加窗      */

                                       /* 为低频数据加窗      */
        energy1 += inst.Tbuf.winData[i] * inst.Tbuf.winData[i];                                     /* 求加窗后的幅度平方和 */
    }

    /* ************** 如果energy1 =0 表示输入音频是静音 *******************************/
    if (energy1 == 0.0f) {
        for (i = 0; i < inst.frameLen; i++) {
            /* 输出低频& 限制上下限 */
            dTmp = inst.syntBuf[i] * ola_normalizer[i];
            if      (dTmp <  _SPL_WORD16_MIN) { dTmp =  _SPL_WORD16_MIN; }
            else if (dTmp >  _SPL_WORD16_MAX) { dTmp =  _SPL_WORD16_MAX; }
            outFrameLB[i] = (short)dTmp;


        }

        for (i = 0; i < (inst.nfft - inst.frameLen); i++) {                                         /* 流方式填充syntBut */
            inst.syntBuf[i] = inst.syntBuf[inst.frameLen + i];                                      /* 前96点流过去旧数据 */
        }
        for (i = 0; i < inst.frameLen; i++) {                                                       /* 后160 点 填充0   */
            inst.syntBuf[inst.nfft - inst.frameLen + i] = (float)0.0f;
        }


        return 0;
    }

    inst.blockInd++; 
    nscore_fft(&(inst.fft_inst), inst.Tbuf.winData, inst.fftData, 0);                         /* fftData存放结果*/


    inst.Tbuf.spec.magn[0] = (float)(fabsf(inst.fftData[0]) + 1.0f);                                  /* 频谱第一点     */                                  
    inst.Tbuf.spec.magn[inst.magnLen - 1] = (float)(fabsf(inst.fftData[1]) + 1.0f);                   /* 频谱最后一点   */

    inst.signalEnergy = (float)(inst.fftData[0] * inst.fftData[0]) +   
                        (float)(inst.fftData[1] * inst.fftData[1]);                                  /* 模值平方和     */
    inst.sumMagn = inst.Tbuf.spec.magn[0] + inst.Tbuf.spec.magn[inst.magnLen - 1];                   /* 模值和         */

    for (i = 1; i < inst.magnLen - 1; i++) {
        fTmp  = inst.fftData[2*i  ] * inst.fftData[2*i  ];
        fTmp += inst.fftData[2*i+1] * inst.fftData[2*i+1];
        inst.signalEnergy += fTmp;                                                                   /* 模值平方和     */
        inst.Tbuf.spec.magn[i] = ((float)sqrtf(fTmp)) + 1.0f;              
        inst.sumMagn += inst.Tbuf.spec.magn[i];                                                      /* 模值和         */                                    
    }


    inst.signalEnergy = inst.signalEnergy / ((float)inst.magnLen);                                   /* 求模值平方均值 */  
    Ns_NoiseEstimation( inst.Tbuf.spec.magn, inst.Tbuf.spec.noise);                                  /* 分位数噪声估计 */

    if (inst.blockInd < END_STARTUP_LONG) {
        inst.featureData5 *= inst.blockInd;
        inst.featureData5 += inst.signalEnergy;
        inst.featureData5 /= (inst.blockInd + 1);
    }

    for (i = 0; i < inst.magnLen; i++) {
        inst.snrLocPost[i] = (float)0.0f;
        if (inst.Tbuf.spec.magn[i] > inst.Tbuf.spec.noise[i]) {
            inst.snrLocPost[i] = inst.Tbuf.spec.magn[i] / (inst.Tbuf.spec.noise[i] + (float)0.0001f) - (float)1.0f;
        }
        inst.previousEstimateStsa[i] = inst.magnPrev[i] / (inst.noisePrev[i] + 
                                        (float)0.0001f) * (inst.smooth[i]);

        inst.snrLocPrior[i] = DD_PR_SNR * inst.previousEstimateStsa[i] + 
                                ((float)1.0f - DD_PR_SNR)* inst.snrLocPost[i];   
    } 
    Ns_ComputeSpectralDifference( inst.Tbuf.spec.magn);
    // if (inst.modelUpdateMode >= 1) {
        //
        inst.modelUpdateCnt--;//
                                    // 
        if (inst.modelUpdateCnt > 0) {
            Ns_FeatureParameterExtraction( 0);
        }
        if (inst.modelUpdateCnt == 0) {
            Ns_FeatureParameterExtraction( 1);
            inst.modelUpdateCnt = inst.modelUpdateFrame;
            //
            // if (inst.modelUpdateMode == 1) {
            //     inst.modelUpdateMode = 0;
            // } else {
                inst.featureData6 = inst.featureData6 / ((float)inst.modelUpdateFrame);
                inst.featureData5 = (float)0.5f * (inst.featureData6 + inst.featureData5);
                inst.featureData6 = (float)0.0f;
            // }
        }
    // }
    Ns_SpeechNoiseProb( inst.probSpeechFinal, inst.snrLocPrior, inst.snrLocPost);

    gammaNoiseTmp = NOISE_UPDATE;
    for (i = 0; i < inst.magnLen; i++) {
        probSpeech = inst.probSpeechFinal[i];
        probNonSpeech = (float)1.0f - probSpeech;
        noiseUpdateTmp =  gammaNoiseTmp * inst.noisePrev[i] + ((float)1.0f - gammaNoiseTmp) 
                        * (probNonSpeech * inst.Tbuf.spec.magn[i] + probSpeech * inst.noisePrev[i]);

        gammaNoiseOld = gammaNoiseTmp;
        gammaNoiseTmp = NOISE_UPDATE;

        if (probSpeech > PROB_RANGE) {
            gammaNoiseTmp = SPEECH_UPDATE;
        }

        if (probSpeech < PROB_RANGE) {
            inst.magnAvgPause[i] += GAMMA_PAUSE * (inst.Tbuf.spec.magn[i] - inst.magnAvgPause[i]);
        }
        /* 更新噪声 */
        if (gammaNoiseTmp == gammaNoiseOld) {
            inst.Tbuf.spec.noise[i] = noiseUpdateTmp;
        } else {                       
            inst.Tbuf.spec.noise[i] = gammaNoiseTmp * inst.noisePrev[i] + ((float)1.0f - gammaNoiseTmp) 
                                * (probNonSpeech * inst.Tbuf.spec.magn[i] + probSpeech * inst.noisePrev[i]);
            if (  inst.Tbuf.spec.noise[i] > noiseUpdateTmp) { 
                inst.Tbuf.spec.noise[i] = noiseUpdateTmp;
            }
        }
    }  

    for (i = 0; i < inst.magnLen; i++) {
        currentEstimateStsa = (float)0.0f;
        if (inst.Tbuf.spec.magn[i] > inst.Tbuf.spec.noise[i]) {
            currentEstimateStsa = inst.Tbuf.spec.magn[i] / (inst.Tbuf.spec.noise[i] + (float)0.0001f) - (float)1.0f;
        }
        snrPrior = DD_PR_SNR * inst.previousEstimateStsa[i] + ((float)1.0f - DD_PR_SNR)*currentEstimateStsa;

        /* overdrive 是一个初始化的值大约为1的值，是用来控制~过估计~的 */
        tmpFloat1 = inst.overdrive + snrPrior;
        inst.smooth[i]  = (float)snrPrior / tmpFloat1;

        /* 限制维纳滤波器系数上下限*/
        if      (inst.smooth[i]  < inst.denoiseBound) { inst.smooth[i]  = inst.denoiseBound; }
        else if (inst.smooth[i]  > (float)1.0f)        { inst.smooth[i]  = 1.0f;               }

        /* 备份噪声谱和幅度谱 */
        inst.noisePrev[i] = inst.Tbuf.spec.noise[i];
        inst.magnPrev[i]  = inst.Tbuf.spec.magn[i];
    }  

    /*********** 应用维纳滤波器系数 准备IFFT 数据**************************************/
    inst.fftData[0] *= inst.smooth[0];
    inst.fftData[1] *= inst.smooth[inst.magnLen - 1];//
    for (i = 1; i < inst.magnLen - 1; i++) {
        inst.fftData[2*i]   *= inst.smooth[i]  ;
        inst.fftData[2*i+1] *= inst.smooth[i]  ;
    }

    nscore_fft(&(inst.ifft_inst), inst.fftData, inst.Tbuf.winData, 1);


    for (i = 0; i < inst.nfft; i++) {
        inst.syntBuf[i] +=  inst.window[i] * inst.Tbuf.winData[i];

    }

    for (i = 0; i < inst.frameLen ; i++) {
        dTmp = inst.syntBuf[i] * ola_normalizer[i];
        if      (dTmp <  _SPL_WORD16_MIN) { dTmp =  _SPL_WORD16_MIN; }
        else if (dTmp >  _SPL_WORD16_MAX) { dTmp =  _SPL_WORD16_MAX; }
        outFrameLB[i] = (short)dTmp;
    }

    for (i = 0; i < (inst.nfft - inst.frameLen); i++) {
        inst.syntBuf[i] = inst.syntBuf[inst.frameLen + i];
    }
    for (i = 0; i < inst.frameLen; i++) {
        inst.syntBuf[inst.nfft - inst.frameLen + i] = (float)0.0f;
    }

    return 0;
}


/***********************************************************************************************************************
* end of file
***********************************************************************************************************************/



/* Public mono adapter. The original NS core is a single-instance algorithm. */
void array_nscore_init(unsigned min_gain_percent) {
    memset(&inst, 0, sizeof(inst));
    Ns_InitCore();
    // The supplied sine window is not constant-overlap-add at a 160-sample
    // step. Normalize its analysis*synthesis sum to avoid 100-Hz gain ripple.
    for (int i = 0; i < 160; ++i) {
        float gain = inst.window[i] * inst.window[i];
        if (i + 160 < 256) gain += inst.window[i + 160] * inst.window[i + 160];
        ola_normalizer[i] = 1.0f / gain;
    }
    if (min_gain_percent < 1) min_gain_percent = 1;
    if (min_gain_percent > 100) min_gain_percent = 100;
    inst.denoiseBound = min_gain_percent / 100.0f;
}

void array_nscore_process(const int16_t input[160], int16_t output[160]) {
    if (inst.denoiseBound >= 1.0f) {
        memcpy(output, input, 160 * sizeof(*output));
        return;
    }
    Ns_ProcessCore((short *)input, NULL, output, NULL);
}
