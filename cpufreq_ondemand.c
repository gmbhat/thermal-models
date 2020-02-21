/*
 *  drivers/cpufreq/cpufreq_ondemand.c
 *
 *  Copyright (C)  2001 Russell King
 *            (C)  2003 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>.
 *                      Jun Nakajima <jun.nakajima@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cpufreq.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kernel_stat.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/percpu-defs.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/tick.h>
#include <linux/types.h>
#include <linux/cpu.h>

#include "cpufreq_governor.h"




//added for algorithm
#include <linux/platform_data/ina231.h>       // added
#include <linux/platform_data/odroid_fan.h>   // added
#include <linux/platform_data/exynos_thermal.h> //added


/* On-demand governor macros */
#define DEF_FREQUENCY_DOWN_DIFFERENTIAL		(10)
#define DEF_FREQUENCY_UP_THRESHOLD		(80)
#define DEF_SAMPLING_DOWN_FACTOR		(1)
#define MAX_SAMPLING_DOWN_FACTOR		(100000)
#define MICRO_FREQUENCY_DOWN_DIFFERENTIAL	(3)
#define MICRO_FREQUENCY_UP_THRESHOLD		(95)
#define MICRO_FREQUENCY_MIN_SAMPLE_RATE		(10000)
#define MIN_FREQUENCY_UP_THRESHOLD		(11)
#define MAX_FREQUENCY_UP_THRESHOLD		(100)
#define DEFAULT_ALGORITHM_MODE			(0)
#define DATE15_ALGORITHM_MODE			(1)
#define UTIL_BASED_ALGORITHM_MODE		(3)
#define UTIL_BASED_ALGORITHM_MODE_NEW		(2)
#define POWER_UTIL_BASED_ALGORITHM_MODE_ONCE	(4)
#define POWER_UTIL_BASED_ALGORITHM_MODE		(5)
#define MAX_LOOP_COUNT				(6)		// Maximimum number of CPU/GPU frequency decrease in one iteration of ondemand

// Targets

signed long TARGET_DATE = 68000000;

unsigned int Targ = 6800, Targ1 = 68;
//unsigned int Targ = 7000, Targ1 = 70;

static DEFINE_PER_CPU(struct od_cpu_dbs_info_s, od_cpu_dbs_info);
//static void find_next_freq_cpu_gpu(void);
static void find_next_freq_cpu_gpu(unsigned int *next_freq_cpu, unsigned int *next_step_gpu, unsigned int *targ_freq_gpu);
//static void find_next_freq_cpu_gpu_power(unsigned int *next_freq_cpu, unsigned int *next_step_gpu, unsigned int *targ_freq_gpu);
static void find_total_power_next(void);
static void predict_temp_for_1s(void);
static void asu_date_algorithm(unsigned int *next_freq_cpu, unsigned *targ_freq_cpu_ondemand);
static inline unsigned int find_maximum_value(unsigned int val1, unsigned int val2, unsigned int val3, unsigned int val4, unsigned int val5);
static void gsa_dt_dp_algorithm(unsigned int *next_freq_cpu, unsigned int *next_step_gpu, unsigned int *targ_freq_gpu, unsigned int *maximum_temperature_prediction);
static struct od_ops od_ops;

unsigned int b11 = 1646, b12 = 793, b13 = 0,    b14 = 228;
unsigned int b21 = 1143, b22 = 730, b23 = 178,  b24 = 0;
unsigned int b31 = 1440, b32 = 818, b33 = 0,    b34 = 0;
unsigned int b41 = 1160, b42 = 833, b43 = 1112, b44 = 154;
unsigned int b51 = 1332, b52 = 713, b53 = 0,    b54 = 991;

unsigned int a11 = 9935,  a12 = 15,    a13 = 22,   a14 = 14,   a15 = 7;
unsigned int a21 = 70,    a22 = 9913,  a23 = 1,    a24 = 8,    a25 = 0;
unsigned int a31 = 17,    a32 = 22,    a33 = 9911, a34 = 19,   a35 = 30;
unsigned int a41 = 74,    a42 = 0,     a43 = 0,    a44 = 9904, a45 = 3;
unsigned int a51 = 20,    a52 = 19,    a53 = 22,   a54 = 15,   a55 = 9896;

unsigned int temp_freq_little = 1400000;
unsigned int temp_freq_big    = 1700000;

int temp_util_algo = 50;
extern unsigned int gpu_freq_override_flag;
unsigned int Temp_tmp0,Temp_tmp1,Temp_tmp2,Temp_tmp3,Temp_tmp4;
unsigned int Temp_tmp0_next,Temp_tmp1_next,Temp_tmp2_next,Temp_tmp3_next,Temp_tmp4_next;
unsigned int Final_pred0,Final_pred1,Final_pred2,Final_pred3,Final_pred4;
unsigned int P0,P1,P2,P3,P4;
unsigned int cntr_tmp=0,cntr_tmp1=0;

unsigned int c1=100000, c2=100000, targ_freq_cpu, targ_freq_gpu, targ_freq_gpu_temp, a0,a1, a2, a3, a4, a5,a6, a7,cnt5,cnt6,cnt7,cnt8_gpu = 0,cnt9,cnt10;
unsigned int delta_j1=1000000, delta_j2=1000000, delta_p1=1, delta_p2=1;
unsigned int cur_freq_cpu=1200000, cur_freq_gpu=177;
unsigned int c1_flag=1, c2_flag=1;
unsigned int targ_freq_cpu_ondemand=1200000,targ_freq_gpu_default=177;
unsigned int  temp_max, temp_max_prediction;
unsigned int next_freq_cpu = 0, next_step_gpu = 0;
long next_a15_pow, next_a7_pow, next_mem_pow, next_gpu_pow, next_dynamic_pow;
signed long VVF,VVF1200=1170187,VVF1300=1345898,VVF1400=1485260,VVF1500=1673496,VVF1600=1918440;
signed long VVF1700=2185161,VVF1800=2474561,VVF1900=2845371,VVF2000=3393012,VVF2100=3848542;
unsigned long GVVF_0=116848, GVVF_1=197879, GVVF_2=291430, GVVF_3=389091, GVVF_4=480000, GVVF_5=584489;
long cur_gpu_vvf;
long cur_VVF_global;
unsigned int leakage_pow_global;
#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMAND
static struct cpufreq_governor cpufreq_gov_ondemand;
#endif

static unsigned int default_powersave_bias;

static void ondemand_powersave_bias_init_cpu(int cpu)
{
	struct od_cpu_dbs_info_s *dbs_info = &per_cpu(od_cpu_dbs_info, cpu);

	dbs_info->freq_table = cpufreq_frequency_get_table(cpu);
	dbs_info->freq_lo = 0;
}

/*
 * Not all CPUs want IO time to be accounted as busy; this depends on how
 * efficient idling at a higher frequency/voltage is.
 * Pavel Machek says this is not so for various generations of AMD and old
 * Intel systems.
 * Mike Chan (android.com) claims this is also not true for ARM.
 * Because of this, whitelist specific known (series) of CPUs by default, and
 * leave all others up to the user.
 */
static int should_io_be_busy(void)
{
#if defined(CONFIG_X86)
	/*
	 * For Intel, Core 2 (model 15) and later have an efficient idle.
	 */
	if (boot_cpu_data.x86_vendor == X86_VENDOR_INTEL &&
			boot_cpu_data.x86 == 6 &&
			boot_cpu_data.x86_model >= 15)
		return 1;
#endif
	return 0;
}

/*
 * Find right freq to be set now with powersave_bias on.
 * Returns the freq_hi to be used right now and will set freq_hi_jiffies,
 * freq_lo, and freq_lo_jiffies in percpu area for averaging freqs.
 */
static unsigned int generic_powersave_bias_target(struct cpufreq_policy *policy,
		unsigned int freq_next, unsigned int relation)
{
	unsigned int freq_req, freq_reduc, freq_avg;
	unsigned int freq_hi, freq_lo;
	unsigned int index = 0;
	unsigned int jiffies_total, jiffies_hi, jiffies_lo;
	struct od_cpu_dbs_info_s *dbs_info = &per_cpu(od_cpu_dbs_info,
			policy->cpu);
	struct dbs_data *dbs_data = policy->governor_data;
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;

	if (!dbs_info->freq_table) {
		dbs_info->freq_lo = 0;
		dbs_info->freq_lo_jiffies = 0;
		return freq_next;
	}

	cpufreq_frequency_table_target(policy, dbs_info->freq_table, freq_next,
			relation, &index);
	freq_req = dbs_info->freq_table[index].frequency;
	freq_reduc = freq_req * od_tuners->powersave_bias / 1000;
	freq_avg = freq_req - freq_reduc;

	/* Find freq bounds for freq_avg in freq_table */
	index = 0;
	cpufreq_frequency_table_target(policy, dbs_info->freq_table, freq_avg,
			CPUFREQ_RELATION_H, &index);
	freq_lo = dbs_info->freq_table[index].frequency;
	index = 0;
	cpufreq_frequency_table_target(policy, dbs_info->freq_table, freq_avg,
			CPUFREQ_RELATION_L, &index);
	freq_hi = dbs_info->freq_table[index].frequency;

	/* Find out how long we have to be in hi and lo freqs */
	if (freq_hi == freq_lo) {
		dbs_info->freq_lo = 0;
		dbs_info->freq_lo_jiffies = 0;
		return freq_lo;
	}
	jiffies_total = usecs_to_jiffies(od_tuners->sampling_rate);
	jiffies_hi = (freq_avg - freq_lo) * jiffies_total;
	jiffies_hi += ((freq_hi - freq_lo) / 2);
	jiffies_hi /= (freq_hi - freq_lo);
	jiffies_lo = jiffies_total - jiffies_hi;
	dbs_info->freq_lo = freq_lo;
	dbs_info->freq_lo_jiffies = jiffies_lo;
	dbs_info->freq_hi_jiffies = jiffies_hi;
	return freq_hi;
}

static void ondemand_powersave_bias_init(void)
{
	int i;
	for_each_online_cpu(i) {
		ondemand_powersave_bias_init_cpu(i);
	}
}

static void dbs_freq_increase(struct cpufreq_policy *p, unsigned int freq)
{
	struct dbs_data *dbs_data = p->governor_data;
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;

	if (od_tuners->powersave_bias)
		freq = od_ops.powersave_bias_target(p, freq,
				CPUFREQ_RELATION_H);
	//else if (p->cur == p->max)
	//	return;
	//if(p->cpu==0)
	//	__cpufreq_driver_target(p, freq, od_tuners->powersave_bias ?
	//			CPUFREQ_RELATION_L : CPUFREQ_RELATION_H);

	//else
	targ_freq_cpu_ondemand = freq;

}

/*
 * Every sampling_rate, we check, if current idle time is less than 20%
 * (default), then we try to increase frequency. Every sampling_rate, we look
 * for the lowest frequency which can sustain the load while keeping idle time
 * over 30%. If such a frequency exist, we try to decrease to this frequency.
 *
 * Any frequency increase takes it to the maximum frequency. Frequency reduction
 * happens at minimum steps of 5% (default) of current frequency
 */
static void od_check_cpu(int cpu, unsigned int load_freq)
{
	struct od_cpu_dbs_info_s *dbs_info = &per_cpu(od_cpu_dbs_info, cpu);
	struct cpufreq_policy *policy = dbs_info->cdbs.cur_policy;
	struct dbs_data *dbs_data = policy->governor_data;
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	int cnt;
	unsigned int current_algorithm = DEFAULT_ALGORITHM_MODE;
	bool is_violation = false;
	bool is_violation_next = false;
	dbs_info->freq_lo = 0;
	///////////////..........Temperature Prediction Algorithm Start...........////////////////


	temp_max = exynos_thermal_get_value();  // Temperature values
	ina231_get_sensor_values();             // Power values

	a7_cur_uW  = a7_cur_uW  / 100;
	a15_cur_uW = a15_cur_uW / 100;
	Mem_cur_uW = Mem_cur_uW / 100;
	GPU_cur_uW = GPU_cur_uW / 100;

	P0 = b11 * a7_cur_uW + b12 * a15_cur_uW + b13 * Mem_cur_uW + b14 * GPU_cur_uW;
	P1 = b21 * a7_cur_uW + b22 * a15_cur_uW + b23 * Mem_cur_uW + b24 * GPU_cur_uW;
	P2 = b31 * a7_cur_uW + b32 * a15_cur_uW + b33 * Mem_cur_uW + b34 * GPU_cur_uW;
	P3 = b41 * a7_cur_uW + b42 * a15_cur_uW + b43 * Mem_cur_uW + b44 * GPU_cur_uW;
	P4 = b51 * a7_cur_uW + b52 * a15_cur_uW + b53 * Mem_cur_uW + b54 * GPU_cur_uW;

	P0 = P0/100;
	P1 = P1/100;
	P2 = P2/100;
	P3 = P3/100;
	P4 = P4/100;

	Temp_tmp0 = Temp_Core0 * 100;
	Temp_tmp1 = Temp_Core1 * 100;
	Temp_tmp2 = Temp_Core2 * 100;
	Temp_tmp3 = Temp_Core3 * 100;
	Temp_tmp4 = Temp_Core4 * 100;

	od_tuners->Temp0      = Temp_Core0;
	od_tuners->Temp1      = Temp_Core1;
	od_tuners->Temp2      = Temp_Core2;
	od_tuners->Temp3      = Temp_Core3;
	od_tuners->Temp4      = Temp_Core4;
	for(cntr_tmp=0;cntr_tmp<10;cntr_tmp++) /////Predict for 1sec
	{
		Final_pred0 = a11 * Temp_tmp0 + a12 * Temp_tmp1 + a13 * Temp_tmp2 + a14 * Temp_tmp3 + a15 * Temp_tmp4 + P0;
		Final_pred1 = a21 * Temp_tmp0 + a22 * Temp_tmp1 + a23 * Temp_tmp2 + a24 * Temp_tmp3 + a25 * Temp_tmp4 + P1;
		Final_pred2 = a31 * Temp_tmp0 + a32 * Temp_tmp1 + a33 * Temp_tmp2 + a34 * Temp_tmp3 + a35 * Temp_tmp4 + P2;
		Final_pred3 = a41 * Temp_tmp0 + a42 * Temp_tmp1 + a43 * Temp_tmp2 + a44 * Temp_tmp3 + a45 * Temp_tmp4 + P3;
		Final_pred4 = a51 * Temp_tmp0 + a52 * Temp_tmp1 + a53 * Temp_tmp2 + a54 * Temp_tmp3 + a55 * Temp_tmp4 + P4;

		Temp_tmp0 = Final_pred0/10000;
		Temp_tmp1 = Final_pred1/10000;
		Temp_tmp2 = Final_pred2/10000;
		Temp_tmp3 = Final_pred3/10000;
		Temp_tmp4 = Final_pred4/10000;
	}

	temp_max_prediction = find_maximum_value(Temp_tmp0, Temp_tmp1, Temp_tmp1, Temp_tmp1, Temp_tmp1);

	if ((temp_max*100) > temp_max_prediction)
		temp_max_prediction = temp_max*100;

	od_tuners->Temp_Pred0 = Temp_tmp0;
	od_tuners->Temp_Pred1 = Temp_tmp1;
	od_tuners->Temp_Pred2 = Temp_tmp2;
	od_tuners->Temp_Pred3 = Temp_tmp3;
	od_tuners->Temp_Pred4 = Temp_tmp4;

	///////////////..........Temperature Prediction Algorithm End...........////////////////

	od_tuners->GPU_util      = temp_util_gpu;
	if(policy->cpu >= 4)
		od_tuners->Big_util   = load_freq/policy->cur;
	else
		od_tuners->Little_util      = load_freq/policy->cur;

	c1 = 10000000 * od_tuners->Big_util;
	c2 = 1000 * od_tuners->GPU_util;


	//	 Check for frequency increase
	if (load_freq > od_tuners->up_threshold * policy->cur) {
		//		/* If switching to max speed, apply sampling_down_factor
		if (policy->cur < policy->max)
			dbs_info->rate_mult =
				od_tuners->sampling_down_factor;
		dbs_freq_increase(policy, policy->max);
		//return;                                                     // algo
	}


	//	 Check for frequency decrease
	//	 if we cannot reduce the frequency anymore, break out early
//	if (policy->cur == policy->min) {
//		unsigned int blank_var = 0;
//	}
	//	return;

	/*
	 * The optimal frequency is the frequency that is the lowest that can
	 * support the current CPU usage without triggering the up policy. To be
	 * safe, we focus 10 points under the threshold.
	 */
	if (load_freq < od_tuners->adj_up_threshold
			* policy->cur) {
		unsigned int freq_next;
		freq_next = load_freq / od_tuners->adj_up_threshold;

		/* No longer fully busy, reset rate_mult */
		dbs_info->rate_mult = 1;

		if (freq_next < policy->min)
			freq_next = policy->min;

		if (!od_tuners->powersave_bias) {
			targ_freq_cpu_ondemand = freq_next;
			goto continue_od;
			//	__cpufreq_driver_target(policy, freq_next,
			//			CPUFREQ_RELATION_L);
			//	return;
		}

		freq_next = od_ops.powersave_bias_target(policy, freq_next,
				CPUFREQ_RELATION_L);
	//	__cpufreq_driver_target(policy, freq_next, CPUFREQ_RELATION_L);
		targ_freq_cpu_ondemand = freq_next;
	}

continue_od:
	if (targ_freq_cpu_ondemand > 2000000)
		targ_freq_cpu_ondemand = 2000000;
	///////////////..........Temperature Control Algorithm Start............////////////////
//	gpu_freq_override_flag = 0;
	targ_step_gpu = cur_gpu_step;
	targ_freq_cpu = targ_freq_cpu_ondemand;
	targ_freq_gpu = 0;

	if (od_tuners->algorithm_mode == UTIL_BASED_ALGORITHM_MODE_NEW) {
		if (od_tuners->GPU_util > 0)
			current_algorithm = UTIL_BASED_ALGORITHM_MODE_NEW;
		else
			current_algorithm = DATE15_ALGORITHM_MODE;
	} else if (od_tuners->algorithm_mode == DATE15_ALGORITHM_MODE)
		current_algorithm = DATE15_ALGORITHM_MODE;

	od_tuners->current_algorithm = current_algorithm;
	is_violation  = (Temp_tmp0 >= Targ) || (Temp_tmp1 >= Targ) || (Temp_tmp2 >= Targ) || (Temp_tmp3 >= Targ) || (temp_max >= Targ1) ||
		(Temp_tmp4 >= Targ);
	is_violation_next = is_violation;

	if (policy->cpu >= 4) {								// Make decisions only when being called from the big cluster
		if ((current_algorithm != DEFAULT_ALGORITHM_MODE) && (is_violation)) {
			unsigned int current_cpu_frequency_actual, maximum_temperature_prediction = temp_max_prediction;
			current_cpu_frequency_actual = policy->cur;
			cur_freq_cpu = targ_freq_cpu_ondemand;
			next_freq_cpu = cur_freq_cpu;				// Current Big cluster CPU
			next_step_gpu = cur_gpu_step;				// Current GPU frequecy from the GPU governor
			Temp_tmp0_next = Temp_tmp0;				// Current temperature predic
			Temp_tmp1_next = Temp_tmp1;
			Temp_tmp2_next = Temp_tmp2;
			Temp_tmp3_next = Temp_tmp3;
			Temp_tmp4_next = Temp_tmp4;
			switch (cur_gpu_step) {
				case 0:
					cur_freq_gpu = 177;
					cur_gpu_vvf = GVVF_0;
					break;
				case 1:
					cur_freq_gpu = 266;
					cur_gpu_vvf = GVVF_1;
					break;
				case 2:
					cur_freq_gpu = 350;
					cur_gpu_vvf = GVVF_2;
					break;
				case 3:
					cur_freq_gpu = 420;
					cur_gpu_vvf = GVVF_3;
					break;
				case 4:
					cur_freq_gpu = 480;
					cur_gpu_vvf = GVVF_4;
					break;
				case 5:
					cur_freq_gpu = 543;
					cur_gpu_vvf = GVVF_5;
					break;
			}
			cnt = 0;
			if (current_algorithm == DATE15_ALGORITHM_MODE) {
				asu_date_algorithm(&next_freq_cpu, &targ_freq_cpu_ondemand);
			} else if (current_algorithm == UTIL_BASED_ALGORITHM_MODE_NEW) {
				// TODO: Add a guard band for CPU and GPU temperature
				// if ((current_cpu_frequency_actual == policy->min) && (cur_gpu_vvf == GVVF_0)
				// && ( != 100))
				//	cpu_down()
				// put hottest core to idle
				while (is_violation_next) {
					find_next_freq_cpu_gpu(&next_freq_cpu, &next_step_gpu, &targ_freq_gpu);
					find_total_power_next();
					predict_temp_for_1s();
					is_violation_next  = (Temp_tmp0_next >= Targ) || (Temp_tmp1_next >= Targ) || (Temp_tmp2_next >= Targ) || (Temp_tmp3_next >= Targ) || 
						(Temp_tmp4_next >= Targ);
					cnt++;
					if (cnt >= MAX_LOOP_COUNT)
						break;
				}
			}
			targ_freq_cpu = next_freq_cpu;
			targ_step_gpu = next_step_gpu;

		} //  if (od_tuners->algorithm_mode != DEFAULT_ALGORITHM_MODE)

	} // if (policy->cpu >= 4)

	od_tuners->gpu_cur = cur_freq_gpu;
	od_tuners->cpu_cur = targ_freq_cpu_ondemand;
	od_tuners->cpu_next = targ_freq_cpu;
	od_tuners->gpu_next = targ_freq_gpu;

	__cpufreq_driver_target(policy, targ_freq_cpu, CPUFREQ_RELATION_L);

}

#if 0
static void gsa_dt_dp_algorithm(unsigned int *next_freq_cpu, unsigned int *next_step_gpu, unsigned int *targ_freq_gpu, unsigned int *maximum_temperature_prediction)
{
	unsigned int delta_perf_cpu = 1, delta_perf_gpu = 1;
	unsigned int target_freq_cpu_temp = 0, target_freq_gpu_temp = 0, target_step_gpu_temp = 0;

	int temperature_prediction_cpu_change = 1, temperature_prediction_gpu_change = 1;			// Initialize to -1
	int cpu_delta_ratio, gpu_delta_ratio;
	int i;
	unsigned int P0_next_cpu, P1_next_cpu, P2_next_cpu, P3_next_cpu, P4_next_cpu;
	unsigned int P0_next_gpu, P1_next_gpu, P2_next_gpu, P3_next_gpu, P4_next_gpu;
	unsigned int Final_pred0_cpu, Final_pred1_cpu, Final_pred2_cpu, Final_pred3_cpu, Final_pred4_cpu;
	unsigned int Final_pred0_gpu, Final_pred1_gpu, Final_pred2_gpu, Final_pred3_gpu, Final_pred4_gpu;
	unsigned int Temp_tmp0_gpu, Temp_tmp1_gpu, Temp_tmp2_gpu,Temp_tmp3_gpu,Temp_tmp4_gpu;
	unsigned int Temp_tmp0_cpu, Temp_tmp1_cpu, Temp_tmp2_cpu,Temp_tmp3_cpu,Temp_tmp4_cpu;
	unsigned int dynamic_pow, next_dynamic_power, next_a15_power, a15_cur_uW_scaled, cpu_power_delta, gpu_power_delta;

	unsigned int cpu_change_max_temperature, gpu_change_max_temperature;
	long next_VVF, next_gpu_power, next_gpu_vvf=GVVF_0;

	long temp_gpu_power;
	if(*next_freq_cpu <= 1200000 ) {				// Already at lowest frequency so can't change CPU
		delta_perf_cpu = 1000;
		temperature_prediction_cpu_change = -1000000;
	} else {
		target_freq_cpu_temp = (*next_freq_cpu - 100000);
		delta_perf_cpu = c1/target_freq_cpu_temp - c1/cur_freq_cpu + 1;
	}

	// Find delta perf_gpu
	if (gpu_freq_override_flag == 1) {				// If gpu_freq_over_flag is one it means that our previous decision has not yet been applied. do nothing 
		target_step_gpu_temp = *next_step_gpu;
		delta_perf_gpu = 1;					// Set delta perf to 1 (small value) because we won't change the frequency
		temperature_prediction_gpu_change = -1000000;			// Set temperature prediction to zero because we won't change the frequency of GPU
	} else {
		if(*next_step_gpu == 0) {
			delta_perf_gpu = 1;
			target_step_gpu_temp = 0;
			temperature_prediction_gpu_change = 0;			// Set temperature prediction to zero because we are already at the lowest frequency
		} else if(*next_step_gpu == 1) {
			target_freq_gpu_temp = 177;
			delta_perf_gpu = c2/target_freq_gpu_temp - c2/cur_freq_gpu;
			target_step_gpu_temp = 0;
		} else if(*next_step_gpu == 2) {
			target_freq_gpu_temp = 266;
			delta_perf_gpu = c2/target_freq_gpu_temp - c2/cur_freq_gpu;
			target_step_gpu_temp = 1;
		} else if(*next_step_gpu == 3) {
			target_freq_gpu_temp = 350;
			delta_perf_gpu = c2/target_freq_gpu_temp - c2/cur_freq_gpu;
			target_step_gpu_temp = 2;
		} else if(*next_step_gpu == 4) {
			target_freq_gpu_temp = 420;
			delta_perf_gpu = c2/target_freq_gpu_temp - c2/cur_freq_gpu;
			target_step_gpu_temp = 3;
		} else if(*next_step_gpu == 5) {
			target_freq_gpu_temp = 480;
			delta_perf_gpu = c2/target_freq_gpu_temp - c2/cur_freq_gpu;
			target_step_gpu_temp = 4;
		}
	}

	// Check if either of delta_t is zero. If yes then we can return from here and don't need to update the temperature predictions
	if ((temperature_prediction_cpu_change == -1000000) && (temperature_prediction_gpu_change == -1000000)) {         // Already at lowest freq. No chance
		return;
	}

	// Calculate CPU power if CPU frequency were to change
	

	dynamic_pow = a15_cur_uW - leakage_pow_global;

	if(target_freq_cpu_temp==2100000)
		next_VVF = VVF2100;
	else if(target_freq_cpu_temp==2000000)
		next_VVF = VVF2000;
	else if(target_freq_cpu_temp==1900000)
		next_VVF = VVF1900;
	else if(target_freq_cpu_temp==1800000)
		next_VVF = VVF1800;
	else if(target_freq_cpu_temp==1700000)
		next_VVF = VVF1700;
	else if(target_freq_cpu_temp==1600000)
		next_VVF = VVF1600;
	else if(target_freq_cpu_temp==1500000)
		next_VVF = VVF1500;
	else if(target_freq_cpu_temp==1400000)
		next_VVF = VVF1400;
	else if(target_freq_cpu_temp==1300000)
		next_VVF = VVF1300;
	else
		next_VVF = VVF1200;


	dynamic_pow = dynamic_pow/100;
	next_dynamic_power  = (dynamic_pow * next_VVF)/cur_VVF_global;
	next_dynamic_power = next_dynamic_power *100;
	next_a15_power = next_dynamic_power + leakage_pow_global;
	next_a15_power = next_a15_power/100;

	a15_cur_uW_scaled = a15_cur_uW/100;
	cpu_power_delta = a15_cur_uW_scaled - next_a15_power;
	////////////////////...........GPU.......////////////

	if(*next_step_gpu == 0) {
		next_gpu_vvf = GVVF_0;
	} else if(*next_step_gpu == 1) {
		next_gpu_vvf = GVVF_0;
	} else if(*next_step_gpu == 2) {
		next_gpu_vvf = GVVF_1;
	} else if(*next_step_gpu == 3) {
		next_gpu_vvf = GVVF_2;
	} else if(*next_step_gpu == 4) {
		next_gpu_vvf = GVVF_3;
	} else if(*next_step_gpu == 5) {
		next_gpu_vvf = GVVF_4;
	}

	temp_gpu_power = GPU_cur_uW/100;
	next_gpu_power = (temp_gpu_power * next_gpu_vvf)/cur_gpu_vvf;
	gpu_power_delta = temp_gpu_power - next_gpu_power;


	// Use precalculated P0, ..., P4 values and adjust accordingly to reduce computations
	P0_next_cpu = P0 - b12 * a15_cur_uW_scaled + b12 * next_a15_power;
	P1_next_cpu = P1 - b22 * a15_cur_uW_scaled + b22 * next_a15_power;
	P2_next_cpu = P2 - b32 * a15_cur_uW_scaled + b32 * next_a15_power;
	P3_next_cpu = P3 - b42 * a15_cur_uW_scaled + b42 * next_a15_power;
	P4_next_cpu = P4 - b52 * a15_cur_uW_scaled + b52 * next_a15_power;

	P0_next_gpu = P0 - b14 * temp_gpu_power + b14 * next_gpu_power;
	P1_next_gpu = P1 - b24 * temp_gpu_power + b24 * next_gpu_power;
	P2_next_gpu = P2 - b34 * temp_gpu_power + b34 * next_gpu_power;
	P3_next_gpu = P3 - b44 * temp_gpu_power + b44 * next_gpu_power;
	P4_next_gpu = P4 - b54 * temp_gpu_power + b54 * next_gpu_power;
	
	Temp_tmp0_cpu = Temp_Core0 * 100;
	Temp_tmp1_cpu = Temp_Core1 * 100;
	Temp_tmp2_cpu = Temp_Core2 * 100;
	Temp_tmp3_cpu = Temp_Core3 * 100;
	Temp_tmp4_cpu = Temp_Core4 * 100;

	Temp_tmp0_gpu = Temp_tmp0_cpu;
	Temp_tmp1_gpu = Temp_tmp1_cpu;
	Temp_tmp2_gpu = Temp_tmp2_cpu;
	Temp_tmp3_gpu = Temp_tmp3_cpu;
	Temp_tmp4_gpu = Temp_tmp4_cpu;
	
	for( i=0; i<10; i++) /////Predict for 1sec
	{
		Final_pred0_cpu = a11 * Temp_tmp0_cpu + a12 * Temp_tmp1_cpu
			+ a13 * Temp_tmp2_cpu + a14 * Temp_tmp3_cpu + a15 * Temp_tmp4_cpu + P0_next_cpu;
		Final_pred1_cpu = a21 * Temp_tmp0_cpu + a22 * Temp_tmp1_cpu
			+ a23 * Temp_tmp2_cpu + a24 * Temp_tmp3_cpu + a25 * Temp_tmp4_cpu + P1_next_cpu;
		Final_pred2_cpu = a31 * Temp_tmp0_cpu + a32 * Temp_tmp1_cpu
			+ a33 * Temp_tmp2_cpu + a34 * Temp_tmp3_cpu + a35 * Temp_tmp4_cpu + P2_next_cpu;
		Final_pred3_cpu = a41 * Temp_tmp0_cpu + a42 * Temp_tmp1_cpu
			+ a43 * Temp_tmp2_cpu + a44 * Temp_tmp3_cpu + a45 * Temp_tmp4_cpu + P3_next_cpu;
		Final_pred4_cpu = a51 * Temp_tmp0_cpu + a52 * Temp_tmp1_cpu
			+ a53 * Temp_tmp2_cpu + a54 * Temp_tmp3_cpu + a55 * Temp_tmp4_cpu + P4_next_cpu;

		Temp_tmp0_cpu = Final_pred0_cpu/10000;
		Temp_tmp1_cpu = Final_pred1_cpu/10000;
		Temp_tmp2_cpu = Final_pred2_cpu/10000;
		Temp_tmp3_cpu = Final_pred3_cpu/10000;
		Temp_tmp4_cpu = Final_pred4_cpu/10000;


		Final_pred0_gpu = a11 * Temp_tmp0_gpu + a12 * Temp_tmp1_gpu
			+ a13 * Temp_tmp2_gpu + a14 * Temp_tmp3_gpu + a15 * Temp_tmp4_gpu + P0_next_gpu;
		Final_pred1_gpu = a21 * Temp_tmp0_gpu + a22 * Temp_tmp1_gpu
			+ a23 * Temp_tmp2_gpu + a24 * Temp_tmp3_gpu + a25 * Temp_tmp4_gpu + P1_next_gpu;
		Final_pred2_gpu = a31 * Temp_tmp0_gpu + a32 * Temp_tmp1_gpu
			+ a33 * Temp_tmp2_gpu + a34 * Temp_tmp3_gpu + a35 * Temp_tmp4_gpu + P2_next_gpu;
		Final_pred3_gpu = a41 * Temp_tmp0_gpu + a42 * Temp_tmp1_gpu
			+ a43 * Temp_tmp2_gpu + a44 * Temp_tmp3_gpu + a45 * Temp_tmp4_gpu + P3_next_gpu;
		Final_pred4_gpu = a51 * Temp_tmp0_gpu + a52 * Temp_tmp1_gpu
			+ a53 * Temp_tmp2_gpu + a54 * Temp_tmp3_gpu + a55 * Temp_tmp4_gpu + P4_next_gpu;

		Temp_tmp0_gpu = Final_pred0_gpu/10000;
		Temp_tmp1_gpu = Final_pred1_gpu/10000;
		Temp_tmp2_gpu = Final_pred2_gpu/10000;
		Temp_tmp3_gpu = Final_pred3_gpu/10000;
		Temp_tmp4_gpu = Final_pred4_gpu/10000;
	}
	// Find maximum prediction due to cpu power change
	cpu_change_max_temperature = find_maximum_value(Temp_tmp0_cpu, Temp_tmp1_cpu, Temp_tmp2_cpu, Temp_tmp3_cpu, Temp_tmp4_cpu);
	gpu_change_max_temperature = find_maximum_value(Temp_tmp0_gpu, Temp_tmp1_gpu, Temp_tmp2_gpu, Temp_tmp3_gpu, Temp_tmp4_gpu);

	if (temperature_prediction_cpu_change > 0)
		temperature_prediction_cpu_change = temp_max_prediction - (int)cpu_change_max_temperature;
	if (temperature_prediction_gpu_change > 0)
		temperature_prediction_gpu_change = temp_max_prediction - (int)gpu_change_max_temperature;

	cpu_delta_ratio = (temperature_prediction_cpu_change*100)/(int)delta_perf_cpu;
	gpu_delta_ratio = (temperature_prediction_gpu_change*100)/(int)delta_perf_gpu;

	if(cpu_delta_ratio > gpu_delta_ratio ) {                         // Change frequency of CPU
		*next_freq_cpu = target_freq_cpu_temp;
		*maximum_temperature_prediction = cpu_change_max_temperature;
	} else if (gpu_delta_ratio > cpu_delta_ratio){                                         // Change frequency of GPU
		//printk("oopsy doopsy\n");
		gpu_freq_override_flag = 1;
		*next_step_gpu = target_step_gpu_temp;
		*targ_freq_gpu = target_freq_gpu_temp;
		*maximum_temperature_prediction = gpu_change_max_temperature;
	}
	return;
}
#endif

static inline unsigned int find_maximum_value(unsigned int val1, unsigned int val2, unsigned int val3, unsigned int val4, unsigned int val5)
{
	unsigned int max1 = 0, max2 = 0, maxval = 0;

	if (val1 > val2)
		max1 = val1;
	else
		max1 = val2;

	if (val3 > val4)
		max2 = val3;
	else
		max2 = val4;

	if (max1 > max2)
		maxval = max1;
	else
		maxval = max2;

	if (val5 > maxval)
		maxval = val5;

	return maxval;
}

static void predict_temp_for_1s(void)
{
	int i;
	unsigned int P0_next, P1_next, P2_next, P3_next, P4_next;
	unsigned int Final_pred0_next, Final_pred1_next, Final_pred2_next, Final_pred3_next, Final_pred4_next;
	P0_next = b11 * next_a7_pow + b12 * next_a15_pow + b13 * next_mem_pow + b14 * next_gpu_pow;
	P1_next = b21 * next_a7_pow + b22 * next_a15_pow + b23 * next_mem_pow + b24 * next_gpu_pow;
	P2_next  = b31 * next_a7_pow + b32 * next_a15_pow + b33 * next_mem_pow + b34 * next_gpu_pow;
	P3_next = b41 * next_a7_pow + b42 * next_a15_pow + b43 * next_mem_pow + b44 * next_gpu_pow;
	P4_next = b51 * next_a7_pow + b52 * next_a15_pow + b53 * next_mem_pow + b54 * next_gpu_pow;

	P0_next = P0_next/100;
	P1_next = P1_next/100;
	P2_next = P2_next/100;
	P3_next = P3_next/100;
	P4_next = P4_next/100;

	Temp_tmp0_next = Temp_Core0 * 100;
	Temp_tmp1_next = Temp_Core1 * 100;
	Temp_tmp2_next = Temp_Core2 * 100;
	Temp_tmp3_next = Temp_Core3 * 100;
	Temp_tmp4_next = Temp_Core4 * 100;

	for( i=0; i<10; i++) /////Predict for 1sec
	{
		Final_pred0_next = a11 * Temp_tmp0_next + a12 * Temp_tmp1_next
			+ a13 * Temp_tmp2_next + a14 * Temp_tmp3_next + a15 * Temp_tmp4_next + P0_next;
		Final_pred1_next = a21 * Temp_tmp0_next + a22 * Temp_tmp1_next
			+ a23 * Temp_tmp2_next + a24 * Temp_tmp3_next + a25 * Temp_tmp4_next + P1_next;
		Final_pred2_next = a31 * Temp_tmp0_next + a32 * Temp_tmp1_next
			+ a33 * Temp_tmp2_next + a34 * Temp_tmp3_next + a35 * Temp_tmp4_next + P2_next;
		Final_pred3_next = a41 * Temp_tmp0_next + a42 * Temp_tmp1_next
			+ a43 * Temp_tmp2_next + a44 * Temp_tmp3_next + a45 * Temp_tmp4_next + P3_next;
		Final_pred4_next = a51 * Temp_tmp0_next + a52 * Temp_tmp1_next
			+ a53 * Temp_tmp2_next + a54 * Temp_tmp3_next + a55 * Temp_tmp4_next + P4_next;

		Temp_tmp0_next = Final_pred0_next/10000;
		Temp_tmp1_next = Final_pred1_next/10000;
		Temp_tmp2_next = Final_pred2_next/10000;
		Temp_tmp3_next = Final_pred3_next/10000;
		Temp_tmp4_next = Final_pred4_next/10000;
	}
	//printk("Temp next is %d\n", Temp_tmp0_next);
	return;
}

static void find_total_power_next(void)
{
	unsigned int leakage_pow, dynamic_pow;
	long cur_VVF, next_VVF,next_gpu_power, next_gpu_vvf=GVVF_0;

	long temp_gpu_power;
	if(temp_max >=40 && temp_max < 50)
		leakage_pow  = 519;
	else if(temp_max >=50 && temp_max < 60)
		leakage_pow  = 678;
	else if(temp_max >=60 && temp_max < 70)
		leakage_pow  = 1061;
	else if(temp_max >=70 && temp_max < 80)
		leakage_pow  = 1375;
	else
		leakage_pow  = 1764;

	dynamic_pow = a15_cur_uW - leakage_pow;

	if(next_freq_cpu==2100000)
		next_VVF = VVF2100;
	else if(next_freq_cpu==2000000)
		next_VVF = VVF2000;
	else if(next_freq_cpu==1900000)
		next_VVF = VVF1900;
	else if(next_freq_cpu==1800000)
		next_VVF = VVF1800;
	else if(next_freq_cpu==1700000)
		next_VVF = VVF1700;
	else if(next_freq_cpu==1600000)
		next_VVF = VVF1600;
	else if(next_freq_cpu==1500000)
		next_VVF = VVF1500;
	else if(next_freq_cpu==1400000)
		next_VVF = VVF1400;
	else if(next_freq_cpu==1300000)
		next_VVF = VVF1300;
	else
		next_VVF = VVF1200;

	if(cur_freq_cpu==2100000)
		cur_VVF = VVF2100;
	else if(cur_freq_cpu==2000000)
		cur_VVF = VVF2000;
	else if(cur_freq_cpu==1900000)
		cur_VVF = VVF1900;
	else if(cur_freq_cpu==1800000)
		cur_VVF = VVF1800;
	else if(cur_freq_cpu==1700000)
		cur_VVF = VVF1700;
	else if(cur_freq_cpu==1600000)
		cur_VVF = VVF1600;
	else if(cur_freq_cpu==1500000)
		cur_VVF = VVF1500;
	else if(cur_freq_cpu==1400000)
		cur_VVF = VVF1400;
	else if(cur_freq_cpu==1300000)
		cur_VVF = VVF1300;
	else
		cur_VVF = VVF1200;

	dynamic_pow = dynamic_pow/100;
	next_dynamic_pow  = (dynamic_pow * next_VVF)/cur_VVF;
	next_dynamic_pow = next_dynamic_pow *100;
	next_a15_pow = next_dynamic_pow + leakage_pow;
	next_a7_pow = a7_cur_uW;
	next_mem_pow = Mem_cur_uW;

	////////////////////...........GPU.......////////////

	if(next_step_gpu == 0) {
		next_gpu_vvf = GVVF_0;
	} else if(next_step_gpu == 1) {
		next_gpu_vvf = GVVF_0;
	} else if(next_step_gpu == 2) {
		next_gpu_vvf = GVVF_1;
	} else if(next_step_gpu == 3) {
		next_gpu_vvf = GVVF_2;
	} else if(next_step_gpu == 4) {
		next_gpu_vvf = GVVF_3;
	} else if(next_step_gpu == 5) {
		next_gpu_vvf = GVVF_4;
	}

	temp_gpu_power = GPU_cur_uW/100;
	next_gpu_power = (temp_gpu_power * next_gpu_vvf)/cur_gpu_vvf;
	next_gpu_power = next_gpu_power*100;

	next_gpu_pow = next_gpu_power;
	return;
}

static void find_next_freq_cpu_gpu(unsigned int *next_freq_cpu, unsigned int *next_step_gpu, unsigned int *targ_freq_gpu)
{
	unsigned int targ_freq_gpu_temp = 0, targ_freq_cpu_temp = 0, targ_step_gpu_temp = 0;
	if(*next_freq_cpu <= 1200000 ) {
		delta_j1 = 1000;
	} else {
		targ_freq_cpu_temp = (*next_freq_cpu - 100000);
		delta_j1 = c1/targ_freq_cpu_temp - c1/cur_freq_cpu;
	}

	////////////////////...........GPU.......////////////

	if (gpu_freq_override_flag == 1) {
		targ_step_gpu_temp = *next_step_gpu;
		delta_j2 = 0;
	}
	else
	{
		if(*next_step_gpu == 0) {
			delta_j2 = 1000;
			targ_step_gpu_temp = 0;
		} else if(*next_step_gpu == 1) {
			targ_freq_gpu_temp = 177;
			delta_j2 = c2/targ_freq_gpu_temp - c2/cur_freq_gpu;
			targ_step_gpu_temp = 0;
		} else if(*next_step_gpu == 2) {
			targ_freq_gpu_temp = 266;
			delta_j2 = c2/targ_freq_gpu_temp - c2/cur_freq_gpu;
			targ_step_gpu_temp = 1;
		} else if(*next_step_gpu == 3) {
			targ_freq_gpu_temp = 350;
			delta_j2 = c2/targ_freq_gpu_temp - c2/cur_freq_gpu;
			targ_step_gpu_temp = 2;
		} else if(*next_step_gpu == 4) {
			targ_freq_gpu_temp = 420;
			delta_j2 = c2/targ_freq_gpu_temp - c2/cur_freq_gpu;
			targ_step_gpu_temp = 3;
		} else if(*next_step_gpu == 5) {
			targ_freq_gpu_temp = 480;
			delta_j2 = c2/targ_freq_gpu_temp - c2/cur_freq_gpu;
			targ_step_gpu_temp = 4;
		}
	}
	if ((delta_j1 == 1000) && (delta_j2 == 1000)) {		// Already at lowest freq. No chance
		return;
	}
	else if(delta_j1 < delta_j2 ) {				// Change frequency of CPU
		*next_freq_cpu = targ_freq_cpu_temp;
	} else if ((delta_j1 > delta_j2)){						// Change frequency of GPU
		gpu_freq_override_flag = 1;
		*next_step_gpu = targ_step_gpu_temp;
		*targ_freq_gpu = targ_freq_gpu_temp;
	}

}

#if 0
static void find_next_freq_cpu_gpu_power(unsigned int *next_freq_cpu, unsigned int *next_step_gpu, unsigned int *targ_freq_gpu)
{
	long cur_cpu_vvf, next_cpu_vvf, next_gpu_vvf=GVVF_0;
	unsigned int leakage_pow, dynamic_pow, next_a15_power, next_dynamic_power, targ_freq_cpu_temp=0, targ_step_gpu_temp=0;
	long cpu_power_delta, gpu_power_delta, next_gpu_power;
	long cpu_ratio, gpu_ratio, temp_gpu_power;

	// If already at lowest frequency then set delta j1 to a high value
	if(*next_freq_cpu <= 1200000 ) {
		delta_j1 = 1000;
	} else {
		targ_freq_cpu_temp = (*next_freq_cpu - 100000);
		delta_j1 = c1/targ_freq_cpu_temp - c1/cur_freq_cpu;
	}

	if(temp_max >=40 && temp_max < 50)
		leakage_pow  = 519;
	else if(temp_max >=50 && temp_max < 60)
		leakage_pow  = 678;
	else if(temp_max >=60 && temp_max < 70)
		leakage_pow  = 1061;
	else if(temp_max >=70 && temp_max < 80)
		leakage_pow  = 1375;
	else
		leakage_pow  = 1764;

	dynamic_pow = a15_cur_uW - leakage_pow;


	if(*next_freq_cpu==2100000)
		next_cpu_vvf = VVF2100;
	else if(*next_freq_cpu==2000000)
		next_cpu_vvf = VVF2000;
	else if(*next_freq_cpu==1900000)
		next_cpu_vvf = VVF1900;
	else if(*next_freq_cpu==1800000)
		next_cpu_vvf = VVF1800;
	else if(*next_freq_cpu==1700000)
		next_cpu_vvf = VVF1700;
	else if(*next_freq_cpu==1600000)
		next_cpu_vvf = VVF1600;
	else if(*next_freq_cpu==1500000)
		next_cpu_vvf = VVF1500;
	else if(*next_freq_cpu==1400000)
		next_cpu_vvf = VVF1400;
	else if(*next_freq_cpu==1300000)
		next_cpu_vvf = VVF1300;
	else
		next_cpu_vvf = VVF1200;

	if(cur_freq_cpu==2100000)
		cur_cpu_vvf = VVF2100;
	else if(cur_freq_cpu==2000000)
		cur_cpu_vvf = VVF2000;
	else if(cur_freq_cpu==1900000)
		cur_cpu_vvf = VVF1900;
	else if(cur_freq_cpu==1800000)
		cur_cpu_vvf = VVF1800;
	else if(cur_freq_cpu==1700000)
		cur_cpu_vvf = VVF1700;
	else if(cur_freq_cpu==1600000)
		cur_cpu_vvf = VVF1600;
	else if(cur_freq_cpu==1500000)
		cur_cpu_vvf = VVF1500;
	else if(cur_freq_cpu==1400000)
		cur_cpu_vvf = VVF1400;
	else if(cur_freq_cpu==1300000)
		cur_cpu_vvf = VVF1300;
	else
		cur_cpu_vvf = VVF1200;

	dynamic_pow = dynamic_pow/100;
	next_dynamic_power  = (dynamic_pow * next_cpu_vvf)/cur_cpu_vvf;
	next_dynamic_pow = next_dynamic_pow * 100;
	next_a15_power = next_dynamic_power + leakage_pow;

	cpu_power_delta = next_a15_power - a15_cur_uW;
	////////////////////...........GPU.......////////////
	if (gpu_freq_override_flag == 1) {
		targ_step_gpu_temp = *next_step_gpu;
		delta_j2 = 0.001;
	} else {

		if(*next_step_gpu == 0) {
			delta_j2 = 1000;
			targ_step_gpu_temp = 0;
			next_gpu_vvf = GVVF_0;
		} else if(*next_step_gpu == 1) {
			targ_freq_gpu_temp = 177;
			delta_j2 = c2/targ_freq_gpu_temp - c2/cur_freq_gpu;
			targ_step_gpu_temp = 0;
			next_gpu_vvf = GVVF_0;
		} else if(*next_step_gpu == 2) {
			targ_freq_gpu_temp = 266;
			delta_j2 = c2/targ_freq_gpu_temp - c2/cur_freq_gpu;
			targ_step_gpu_temp = 1;
			next_gpu_vvf = GVVF_1;
		} else if(*next_step_gpu == 3) {
			targ_freq_gpu_temp = 350;
			delta_j2 = c2/targ_freq_gpu_temp - c2/cur_freq_gpu;
			targ_step_gpu_temp = 2;
			next_gpu_vvf = GVVF_2;
		} else if(*next_step_gpu == 4) {
			targ_freq_gpu_temp = 420;
			delta_j2 = c2/targ_freq_gpu_temp - c2/cur_freq_gpu;
			targ_step_gpu_temp = 3;
			next_gpu_vvf = GVVF_3;
		} else if(*next_step_gpu == 5) {
			targ_freq_gpu_temp = 480;
			delta_j2 = c2/targ_freq_gpu_temp - c2/cur_freq_gpu;
			targ_step_gpu_temp = 4;
			next_gpu_vvf = GVVF_4;
		}
	}
	temp_gpu_power = GPU_cur_uW/100;
	next_gpu_power = (temp_gpu_power * next_gpu_vvf)/cur_gpu_vvf;
	next_gpu_power = next_gpu_power*100;
	gpu_power_delta = next_gpu_power - GPU_cur_uW;

	cpu_ratio = (-1*cpu_power_delta)/delta_j1;
	gpu_ratio = (-1*gpu_power_delta)/delta_j2;

	next_a15_pow = next_a15_power;
	next_a7_pow = a7_cur_uW;
	next_mem_pow = Mem_cur_uW;
	next_gpu_pow = next_gpu_power;

	if ((cpu_ratio == 0) && (gpu_ratio == 0)) {	// Already at lowest. Do nothing
		return;
	}
	if(cpu_ratio > gpu_ratio) {			// Change freq of CPU
		*next_freq_cpu = targ_freq_cpu_temp;
		//		*next_step_gpu = cur_gpu_step;
		//		gpu_freq_override_flag = 0;
	} else if ((gpu_ratio > cpu_ratio) && (Temp_tmp4_next >= MAX_GPU_TEMPERATURE_PREDICTION
				|| Temp_Core4 >= MAX_GPU_TEMPERATURE)) {					// Change freq of GPU and set override flag
		gpu_freq_override_flag = 1;
		*next_step_gpu = targ_step_gpu_temp;
		*targ_freq_gpu = targ_freq_gpu_temp;
	}

}
#endif
static void asu_date_algorithm(unsigned int *next_freq_cpu, unsigned *targ_freq_cpu_ondemand)
{
	signed long P_budget_max1,P_budget_max0;
	signed long Prev_Temp0, Prev_Temp1, Prev_Temp2, Prev_Temp3, Prev_Temp4;
	signed long  Headroom0, Headroom1, Headroom2, Headroom3, Headroom4;
	signed long dynamic_pow,leakage_pow;
	signed long  P_budget_max;

	signed long P_budget0, P_budget1, P_budget2, P_budget3, P_budget4;
	signed long dynamic_pow1200,dynamic_pow1300,dynamic_pow1400,dynamic_pow1500,dynamic_pow1600, dynamic_pow1700,dynamic_pow1800,dynamic_pow1900,dynamic_pow2000,dynamic_pow2100;
	unsigned int Temp_tmp0_scaled, Temp_tmp1_scaled, Temp_tmp2_scaled, Temp_tmp3_scaled, Temp_tmp4_scaled;
	Temp_tmp0_scaled = Temp_Core0 * 100;   
	Temp_tmp1_scaled = Temp_Core1 * 100;
	Temp_tmp2_scaled = Temp_Core2 * 100;
	Temp_tmp3_scaled = Temp_Core3 * 100;
	Temp_tmp4_scaled = Temp_Core4 * 100;
	
	//printk("gb 0.2 %ld %ld %ld %ld %ld\n", Temp_tmp0_scaled,Temp_tmp1_scaled,Temp_tmp2_scaled,Temp_tmp3_scaled,Temp_tmp4_scaled);
	Prev_Temp0 = a11 * Temp_tmp0_scaled + a12 * Temp_tmp1_scaled + a13 * Temp_tmp2_scaled + a14 * Temp_tmp3_scaled + a15 * Temp_tmp4_scaled ;
	Prev_Temp1 = a21 * Temp_tmp0_scaled + a22 * Temp_tmp1_scaled + a23 * Temp_tmp2_scaled + a24 * Temp_tmp3_scaled + a25 * Temp_tmp4_scaled ;
	Prev_Temp2 = a31 * Temp_tmp0_scaled + a32 * Temp_tmp1_scaled + a33 * Temp_tmp2_scaled + a34 * Temp_tmp3_scaled + a35 * Temp_tmp4_scaled ;
	Prev_Temp3 = a41 * Temp_tmp0_scaled + a42 * Temp_tmp1_scaled + a43 * Temp_tmp2_scaled + a44 * Temp_tmp3_scaled + a45 * Temp_tmp4_scaled ;
	Prev_Temp4 = a51 * Temp_tmp0_scaled + a52 * Temp_tmp1_scaled + a53 * Temp_tmp2_scaled + a54 * Temp_tmp3_scaled + a55 * Temp_tmp4_scaled ;

	Headroom0 = (TARGET_DATE - Prev_Temp0) * 100;
	Headroom1 = (TARGET_DATE - Prev_Temp1) * 100;
	Headroom2 = (TARGET_DATE - Prev_Temp2) * 100;
	Headroom3 = (TARGET_DATE - Prev_Temp3) * 100;
	Headroom4 = (TARGET_DATE - Prev_Temp4) * 100;

	////....... Solve for Power Budget ......///////

	//printk("gb 1 %ld %ld %ld %ld\n", Headroom0, Headroom1, Headroom2, Headroom3, Headroom4);
	//printk("gb 1.1 %u %u %u %u\n", a7_cur_uW,a15_cur_uW,Mem_cur_uW,GPU_cur_uW );
	Headroom0 = Headroom0 - ((b11*a7_cur_uW) + (b13*Mem_cur_uW) + (b14*GPU_cur_uW));
	if(Headroom0 < 0)
		Headroom0 = 0;
	Headroom1 = Headroom1 - ((b21*a7_cur_uW) + (b23*Mem_cur_uW) + (b24*GPU_cur_uW));
	if(Headroom1 < 0)
		Headroom1 = 0;
	Headroom2 = Headroom2 - ((b31*a7_cur_uW) + (b33*Mem_cur_uW) + (b34*GPU_cur_uW));
	if(Headroom2 < 0)
		Headroom2 = 0;
	Headroom3 = Headroom3 - ((b41*a7_cur_uW) + (b43*Mem_cur_uW) + (b44*GPU_cur_uW));
	if(Headroom3 < 0)
		Headroom3 = 0;
	Headroom4 = Headroom4 - ((b51*a7_cur_uW) + (b53*Mem_cur_uW) + (b54*GPU_cur_uW));
	if(Headroom4 < 0)
		Headroom4 = 0;

	//printk("gb 1.2 %ld %ld %ld %ld %ld\n", Headroom0, Headroom1, Headroom2, Headroom3, Headroom4);
	P_budget0 = Headroom0/(b12);
	P_budget1 = Headroom1/(b22);
	P_budget2 = Headroom2/(b32);
	P_budget3 = Headroom3/(b42);
	P_budget4 = Headroom4/(b52);

	//printk("gb 2 %ld %ld %ld %ld %ld\n", P_budget0, P_budget1, P_budget2, P_budget3, P_budget4);
	if(P_budget0 <= P_budget1) 
		P_budget_max0 = P_budget0;
	else 
		P_budget_max0 = P_budget1;


	if(P_budget2 <= P_budget3) 
		P_budget_max1 = P_budget2;
	else 
		P_budget_max1 = P_budget3;

	if(P_budget_max0 <= P_budget_max1) 
		P_budget_max = P_budget_max0;
	else 
		P_budget_max = P_budget_max1;


	if(P_budget_max <= P_budget4) 
		P_budget_max = P_budget_max;
	else 
		P_budget_max = P_budget4;


	if(temp_max >=40 && temp_max < 50)
		leakage_pow  = 519;
	else if(temp_max >=50 && temp_max < 60)
		leakage_pow  = 678;
	else if(temp_max >=60 && temp_max < 70)
		leakage_pow  = 1061;
	else if(temp_max >=70 && temp_max < 80)
		leakage_pow  = 1375;
	else
		leakage_pow  = 1764;

	P_budget_max = P_budget_max - leakage_pow;



	dynamic_pow = a15_cur_uW - leakage_pow;


	if(cur_freq_cpu==2100000)
		VVF = VVF2100;
	else if(cur_freq_cpu==2000000)
		VVF = VVF2000;
	else if(cur_freq_cpu==1900000)
		VVF = VVF1900;
	else if(cur_freq_cpu==1800000)
		VVF = VVF1800;
	else if(cur_freq_cpu==1700000)
		VVF = VVF1700;
	else if(cur_freq_cpu==1600000)
		VVF = VVF1600;
	else if(cur_freq_cpu==1500000)
		VVF = VVF1500;
	else if(cur_freq_cpu==1400000)
		VVF = VVF1400;
	else if(cur_freq_cpu==1300000)
		VVF = VVF1300;
	else
		VVF = VVF1200;

	dynamic_pow = dynamic_pow/100;
	dynamic_pow1200  = (dynamic_pow * VVF1200)/VVF;  
	dynamic_pow1200 = dynamic_pow1200*100;
	dynamic_pow1300  = (dynamic_pow * VVF1300)/VVF; 
	dynamic_pow1300 = dynamic_pow1300*100;
	dynamic_pow1400  = (dynamic_pow * VVF1400)/VVF; 
	dynamic_pow1400 = dynamic_pow1400*100;
	dynamic_pow1500  = (dynamic_pow * VVF1500)/VVF; 
	dynamic_pow1500 = dynamic_pow1500*100;
	dynamic_pow1600  = (dynamic_pow * VVF1600)/VVF; 
	dynamic_pow1600 = dynamic_pow1600*100;
	dynamic_pow1700  = (dynamic_pow * VVF1700)/VVF; 
	dynamic_pow1700 = dynamic_pow1700*100;
	dynamic_pow1800  = (dynamic_pow * VVF1800)/VVF; 
	dynamic_pow1800 = dynamic_pow1800*100;
	dynamic_pow1900  = (dynamic_pow * VVF1900)/VVF; 
	dynamic_pow1900 = dynamic_pow1900*100;
	dynamic_pow2000  = (dynamic_pow * VVF2000)/VVF; 
	dynamic_pow2000 = dynamic_pow2000*100;
	dynamic_pow2100  = (dynamic_pow * VVF2100)/VVF; 
	dynamic_pow2100 = dynamic_pow2100*100;


	dynamic_pow = dynamic_pow*100;
	if(P_budget_max > dynamic_pow2000 && P_budget_max <= dynamic_pow2100)
		*next_freq_cpu = 2000000;
	else if(P_budget_max > dynamic_pow1900 && P_budget_max <= dynamic_pow2000)
		*next_freq_cpu = 1900000;
	else if(P_budget_max > dynamic_pow1800 && P_budget_max <= dynamic_pow1900) 
		*next_freq_cpu = 1800000;
	else if(P_budget_max > dynamic_pow1700 && P_budget_max <= dynamic_pow1800) 
		*next_freq_cpu = 1700000;
	else if(P_budget_max > dynamic_pow1600 && P_budget_max <= dynamic_pow1700) 
		*next_freq_cpu = 1600000;
	else if(P_budget_max > dynamic_pow1500 && P_budget_max <= dynamic_pow1600) 
		*next_freq_cpu = 1500000;
	else if(P_budget_max > dynamic_pow1400  && P_budget_max <= dynamic_pow1500)
		*next_freq_cpu = 1400000;
	else if(P_budget_max > dynamic_pow1300  && P_budget_max <= dynamic_pow1400)
		*next_freq_cpu = 1300000;
	else if(P_budget_max <= dynamic_pow1300) 
		*next_freq_cpu = 1200000;
	else
		*next_freq_cpu = *targ_freq_cpu_ondemand;

	if (*targ_freq_cpu_ondemand < *next_freq_cpu)
		*next_freq_cpu = *targ_freq_cpu_ondemand;

	//printk("power %ld budget %ld cur cpu %u next cpu %u %u temp0 %d\n", dynamic_pow, P_budget_max, cur_freq_cpu,*next_freq_cpu, Temp_tmp0, sizeof(signed long));
	return;
}
static void od_dbs_timer(struct work_struct *work)
{
	struct od_cpu_dbs_info_s *dbs_info =
		container_of(work, struct od_cpu_dbs_info_s, cdbs.work.work);
	unsigned int cpu = dbs_info->cdbs.cur_policy->cpu;
	struct od_cpu_dbs_info_s *core_dbs_info = &per_cpu(od_cpu_dbs_info,
			cpu);
	struct dbs_data *dbs_data = dbs_info->cdbs.cur_policy->governor_data;
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	int delay = 0, sample_type = core_dbs_info->sample_type;
	bool modify_all = true;

	mutex_lock(&core_dbs_info->cdbs.timer_mutex);
	if (!need_load_eval(&core_dbs_info->cdbs, od_tuners->sampling_rate)) {
		modify_all = false;
		goto max_delay;
	}

	/* Common NORMAL_SAMPLE setup */
	core_dbs_info->sample_type = OD_NORMAL_SAMPLE;
	if (sample_type == OD_SUB_SAMPLE) {
		delay = core_dbs_info->freq_lo_jiffies;
		__cpufreq_driver_target(core_dbs_info->cdbs.cur_policy,
				core_dbs_info->freq_lo, CPUFREQ_RELATION_H);
	} else {
		dbs_check_cpu(dbs_data, cpu);
		if (core_dbs_info->freq_lo) {
			/* Setup timer for SUB_SAMPLE */
			core_dbs_info->sample_type = OD_SUB_SAMPLE;
			delay = core_dbs_info->freq_hi_jiffies;
		}
	}

max_delay:
	if (!delay)
		delay = delay_for_sampling_rate(od_tuners->sampling_rate
				* core_dbs_info->rate_mult);

	gov_queue_work(dbs_data, dbs_info->cdbs.cur_policy, delay, modify_all);
	mutex_unlock(&core_dbs_info->cdbs.timer_mutex);
}

/************************** sysfs interface ************************/
static struct common_dbs_data od_dbs_cdata;

/**
 * update_sampling_rate - update sampling rate effective immediately if needed.
 * @new_rate: new sampling rate
 *
 * If new rate is smaller than the old, simply updating
 * dbs_tuners_int.sampling_rate might not be appropriate. For example, if the
 * original sampling_rate was 1 second and the requested new sampling rate is 10
 * ms because the user needs immediate reaction from ondemand governor, but not
 * sure if higher frequency will be required or not, then, the governor may
 * change the sampling rate too late; up to 1 second later. Thus, if we are
 * reducing the sampling rate, we need to make the new value effective
 * immediately.
 */
static void update_sampling_rate(struct dbs_data *dbs_data,
		unsigned int new_rate)
{
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	int cpu;

	od_tuners->sampling_rate = new_rate = max(new_rate,
			dbs_data->min_sampling_rate);

	for_each_online_cpu(cpu) {
		struct cpufreq_policy *policy;
		struct od_cpu_dbs_info_s *dbs_info;
		unsigned long next_sampling, appointed_at;

		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			continue;
		if (policy->governor != &cpufreq_gov_ondemand) {
			cpufreq_cpu_put(policy);
			continue;
		}
		dbs_info = &per_cpu(od_cpu_dbs_info, cpu);
		cpufreq_cpu_put(policy);

		mutex_lock(&dbs_info->cdbs.timer_mutex);

		if (!delayed_work_pending(&dbs_info->cdbs.work)) {
			mutex_unlock(&dbs_info->cdbs.timer_mutex);
			continue;
		}

		next_sampling = jiffies + usecs_to_jiffies(new_rate);
		appointed_at = dbs_info->cdbs.work.timer.expires;

		if (time_before(next_sampling, appointed_at)) {

			mutex_unlock(&dbs_info->cdbs.timer_mutex);
			cancel_delayed_work_sync(&dbs_info->cdbs.work);
			mutex_lock(&dbs_info->cdbs.timer_mutex);

			gov_queue_work(dbs_data, dbs_info->cdbs.cur_policy,
					usecs_to_jiffies(new_rate), true);

		}
		mutex_unlock(&dbs_info->cdbs.timer_mutex);
	}
}

static ssize_t store_sampling_rate(struct dbs_data *dbs_data, const char *buf,
		size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	update_sampling_rate(dbs_data, input);
	return count;
}

static ssize_t store_algorithm_mode(struct dbs_data *dbs_data, const char *buf,
		size_t count)
{
	unsigned int input;
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	od_tuners->algorithm_mode = input;
	return count;
}

static ssize_t store_current_algorithm(struct dbs_data *dbs_data, const char *buf,
		size_t count)
{
	unsigned int input;
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	od_tuners->current_algorithm = input;
	return count;
}

static ssize_t store_target_temperature(struct dbs_data *dbs_data, const char *buf,
		size_t count)
{
	unsigned int input;
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	od_tuners->target_temperature = input;
	Targ = input*100;
	Targ1 = input;
	TARGET_DATE = ((signed long)input)* 1000000L;
	return count;
}
static ssize_t store_gpu_cur(struct dbs_data *dbs_data, const char *buf,
		size_t count)
{
	unsigned int input;
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	od_tuners->gpu_cur = input;
	return count;
}
static ssize_t store_cpu_cur(struct dbs_data *dbs_data, const char *buf,
		size_t count)
{
	unsigned int input;
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	od_tuners->cpu_cur = input;
	return count;
}
static ssize_t store_gpu_next(struct dbs_data *dbs_data, const char *buf,
		size_t count)
{
	unsigned int input;
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	od_tuners->gpu_next = input;
	return count;
}
static ssize_t store_cpu_next(struct dbs_data *dbs_data, const char *buf,
		size_t count)
{
	unsigned int input;
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	od_tuners->cpu_next = input;
	return count;
}
static ssize_t store_io_is_busy(struct dbs_data *dbs_data, const char *buf,
		size_t count)
{
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;
	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	od_tuners->io_is_busy = !!input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct od_cpu_dbs_info_s *dbs_info = &per_cpu(od_cpu_dbs_info,
				j);
		dbs_info->cdbs.prev_cpu_idle = get_cpu_idle_time(j,
				&dbs_info->cdbs.prev_cpu_wall, od_tuners->io_is_busy);
	}
	return count;
}

static ssize_t store_up_threshold(struct dbs_data *dbs_data, const char *buf,
		size_t count)
{
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_FREQUENCY_UP_THRESHOLD ||
			input < MIN_FREQUENCY_UP_THRESHOLD) {
		return -EINVAL;
	}
	/* Calculate the new adj_up_threshold */
	od_tuners->adj_up_threshold += input;
	od_tuners->adj_up_threshold -= od_tuners->up_threshold;

	od_tuners->up_threshold = input;
	return count;
}

static ssize_t store_sampling_down_factor(struct dbs_data *dbs_data,
		const char *buf, size_t count)
{
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	unsigned int input, j;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_SAMPLING_DOWN_FACTOR || input < 1)
		return -EINVAL;
	od_tuners->sampling_down_factor = input;

	/* Reset down sampling multiplier in case it was active */
	for_each_online_cpu(j) {
		struct od_cpu_dbs_info_s *dbs_info = &per_cpu(od_cpu_dbs_info,
				j);
		dbs_info->rate_mult = 1;
	}
	return count;
}

static ssize_t store_ignore_nice_load(struct dbs_data *dbs_data,
		const char *buf, size_t count)
{
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == od_tuners->ignore_nice_load) { /* nothing to do */
		return count;
	}
	od_tuners->ignore_nice_load = input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct od_cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->cdbs.prev_cpu_idle = get_cpu_idle_time(j,
				&dbs_info->cdbs.prev_cpu_wall, od_tuners->io_is_busy);
		if (od_tuners->ignore_nice_load)
			dbs_info->cdbs.prev_cpu_nice =
				kcpustat_cpu(j).cpustat[CPUTIME_NICE];

	}
	return count;
}



static ssize_t store_GPU_temp_pred(struct dbs_data *dbs_data,
		const char *buf, size_t count)
{
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == od_tuners->GPU_temp_pred) { /* nothing to do */
		return count;
	}
	od_tuners->GPU_temp_pred = input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct od_cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->cdbs.prev_cpu_idle = get_cpu_idle_time(j,
				&dbs_info->cdbs.prev_cpu_wall, od_tuners->io_is_busy);
		if (od_tuners->GPU_temp_pred)
			dbs_info->cdbs.prev_cpu_nice =
				kcpustat_cpu(j).cpustat[CPUTIME_NICE];

	}
	return count;
}



static ssize_t store_Temp_Pred0(struct dbs_data *dbs_data,
		const char *buf, size_t count)
{
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == od_tuners->Temp_Pred0) { /* nothing to do */
		return count;
	}
	od_tuners->Temp_Pred0 = input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct od_cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->cdbs.prev_cpu_idle = get_cpu_idle_time(j,
				&dbs_info->cdbs.prev_cpu_wall, od_tuners->io_is_busy);
		if (od_tuners->Temp_Pred0)
			dbs_info->cdbs.prev_cpu_nice =
				kcpustat_cpu(j).cpustat[CPUTIME_NICE];

	}
	return count;
}

static ssize_t store_Temp0(struct dbs_data *dbs_data,
		const char *buf, size_t count)
{
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == od_tuners->Temp0) { /* nothing to do */
		return count;
	}
	od_tuners->Temp0 = input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct od_cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->cdbs.prev_cpu_idle = get_cpu_idle_time(j,
				&dbs_info->cdbs.prev_cpu_wall, od_tuners->io_is_busy);
		if (od_tuners->Temp0)
			dbs_info->cdbs.prev_cpu_nice =
				kcpustat_cpu(j).cpustat[CPUTIME_NICE];

	}
	return count;
}

static ssize_t store_GPU_util(struct dbs_data *dbs_data,
		const char *buf, size_t count)
{
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == od_tuners->GPU_util) { /* nothing to do */
		return count;
	}
	od_tuners->GPU_util = input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct od_cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->cdbs.prev_cpu_idle = get_cpu_idle_time(j,
				&dbs_info->cdbs.prev_cpu_wall, od_tuners->io_is_busy);
		if (od_tuners->GPU_util)
			dbs_info->cdbs.prev_cpu_nice =
				kcpustat_cpu(j).cpustat[CPUTIME_NICE];

	}
	return count;
}


static ssize_t store_Big_util(struct dbs_data *dbs_data,
		const char *buf, size_t count)
{
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == od_tuners->Big_util) { /* nothing to do */
		return count;
	}
	od_tuners->Big_util = input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct od_cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->cdbs.prev_cpu_idle = get_cpu_idle_time(j,
				&dbs_info->cdbs.prev_cpu_wall, od_tuners->io_is_busy);
		if (od_tuners->Big_util)
			dbs_info->cdbs.prev_cpu_nice =
				kcpustat_cpu(j).cpustat[CPUTIME_NICE];

	}
	return count;
}



static ssize_t store_Little_util(struct dbs_data *dbs_data,
		const char *buf, size_t count)
{
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == od_tuners->Little_util) { /* nothing to do */
		return count;
	}
	od_tuners->Little_util = input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct od_cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->cdbs.prev_cpu_idle = get_cpu_idle_time(j,
				&dbs_info->cdbs.prev_cpu_wall, od_tuners->io_is_busy);
		if (od_tuners->Little_util)
			dbs_info->cdbs.prev_cpu_nice =
				kcpustat_cpu(j).cpustat[CPUTIME_NICE];

	}
	return count;
}



static ssize_t store_Temp1(struct dbs_data *dbs_data,
		const char *buf, size_t count)
{
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == od_tuners->Temp1) { /* nothing to do */
		return count;
	}
	od_tuners->Temp1 = input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct od_cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->cdbs.prev_cpu_idle = get_cpu_idle_time(j,
				&dbs_info->cdbs.prev_cpu_wall, od_tuners->io_is_busy);
		if (od_tuners->Temp1)
			dbs_info->cdbs.prev_cpu_nice =
				kcpustat_cpu(j).cpustat[CPUTIME_NICE];

	}
	return count;
}


static ssize_t store_Temp2(struct dbs_data *dbs_data,
		const char *buf, size_t count)
{
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == od_tuners->Temp2) { /* nothing to do */
		return count;
	}
	od_tuners->Temp2 = input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct od_cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->cdbs.prev_cpu_idle = get_cpu_idle_time(j,
				&dbs_info->cdbs.prev_cpu_wall, od_tuners->io_is_busy);
		if (od_tuners->Temp2)
			dbs_info->cdbs.prev_cpu_nice =
				kcpustat_cpu(j).cpustat[CPUTIME_NICE];

	}
	return count;
}


static ssize_t store_Temp3(struct dbs_data *dbs_data,
		const char *buf, size_t count)
{
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == od_tuners->Temp3) { /* nothing to do */
		return count;
	}
	od_tuners->Temp3 = input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct od_cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->cdbs.prev_cpu_idle = get_cpu_idle_time(j,
				&dbs_info->cdbs.prev_cpu_wall, od_tuners->io_is_busy);
		if (od_tuners->Temp3)
			dbs_info->cdbs.prev_cpu_nice =
				kcpustat_cpu(j).cpustat[CPUTIME_NICE];

	}
	return count;
}


static ssize_t store_Temp4(struct dbs_data *dbs_data,
		const char *buf, size_t count)
{
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == od_tuners->Temp4) { /* nothing to do */
		return count;
	}
	od_tuners->Temp4 = input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct od_cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->cdbs.prev_cpu_idle = get_cpu_idle_time(j,
				&dbs_info->cdbs.prev_cpu_wall, od_tuners->io_is_busy);
		if (od_tuners->Temp4)
			dbs_info->cdbs.prev_cpu_nice =
				kcpustat_cpu(j).cpustat[CPUTIME_NICE];

	}
	return count;
}


static ssize_t store_Temp_Pred1(struct dbs_data *dbs_data,
		const char *buf, size_t count)
{
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == od_tuners->Temp_Pred1) { /* nothing to do */
		return count;
	}
	od_tuners->Temp_Pred1 = input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct od_cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->cdbs.prev_cpu_idle = get_cpu_idle_time(j,
				&dbs_info->cdbs.prev_cpu_wall, od_tuners->io_is_busy);
		if (od_tuners->Temp_Pred1)
			dbs_info->cdbs.prev_cpu_nice =
				kcpustat_cpu(j).cpustat[CPUTIME_NICE];

	}
	return count;
}


static ssize_t store_Temp_Pred2(struct dbs_data *dbs_data,
		const char *buf, size_t count)
{
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == od_tuners->Temp_Pred2) { /* nothing to do */
		return count;
	}
	od_tuners->Temp_Pred2 = input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct od_cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->cdbs.prev_cpu_idle = get_cpu_idle_time(j,
				&dbs_info->cdbs.prev_cpu_wall, od_tuners->io_is_busy);
		if (od_tuners->Temp_Pred2)
			dbs_info->cdbs.prev_cpu_nice =
				kcpustat_cpu(j).cpustat[CPUTIME_NICE];

	}
	return count;
}


static ssize_t store_Temp_Pred3(struct dbs_data *dbs_data,
		const char *buf, size_t count)
{
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == od_tuners->Temp_Pred3) { /* nothing to do */
		return count;
	}
	od_tuners->Temp_Pred3 = input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct od_cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->cdbs.prev_cpu_idle = get_cpu_idle_time(j,
				&dbs_info->cdbs.prev_cpu_wall, od_tuners->io_is_busy);
		if (od_tuners->Temp_Pred3)
			dbs_info->cdbs.prev_cpu_nice =
				kcpustat_cpu(j).cpustat[CPUTIME_NICE];

	}
	return count;
}


static ssize_t store_Temp_Pred4(struct dbs_data *dbs_data,
		const char *buf, size_t count)
{
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == od_tuners->Temp_Pred4) { /* nothing to do */
		return count;
	}
	od_tuners->Temp_Pred4 = input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct od_cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->cdbs.prev_cpu_idle = get_cpu_idle_time(j,
				&dbs_info->cdbs.prev_cpu_wall, od_tuners->io_is_busy);
		if (od_tuners->Temp_Pred4)
			dbs_info->cdbs.prev_cpu_nice =
				kcpustat_cpu(j).cpustat[CPUTIME_NICE];

	}
	return count;
}




static ssize_t store_powersave_bias(struct dbs_data *dbs_data, const char *buf,
		size_t count)
{
	struct od_dbs_tuners *od_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	if (input > 1000)
		input = 1000;

	od_tuners->powersave_bias = input;
	ondemand_powersave_bias_init();
	return count;
}

show_store_one(od, sampling_rate);
show_store_one(od, algorithm_mode);
show_store_one(od, current_algorithm);
show_store_one(od, target_temperature);
show_store_one(od, io_is_busy);
show_store_one(od, up_threshold);
show_store_one(od, sampling_down_factor);
show_store_one(od, ignore_nice_load);
show_store_one(od, powersave_bias);
show_store_one(od, Temp_Pred0);
show_store_one(od, Temp_Pred1);
show_store_one(od, Temp_Pred2);
show_store_one(od, Temp_Pred3);
show_store_one(od, Temp_Pred4);
show_store_one(od, Temp0);
show_store_one(od, Temp1);
show_store_one(od, Temp2);
show_store_one(od, Temp3);
show_store_one(od, Temp4);
show_store_one(od, GPU_util);
show_store_one(od, Big_util);
show_store_one(od, Little_util);
show_store_one(od, GPU_temp_pred);
declare_show_sampling_rate_min(od);
show_store_one(od, gpu_cur);
show_store_one(od, cpu_cur);
show_store_one(od, gpu_next);
show_store_one(od, cpu_next);

gov_sys_pol_attr_rw(sampling_rate);
gov_sys_pol_attr_rw(algorithm_mode);
gov_sys_pol_attr_rw(current_algorithm);
gov_sys_pol_attr_rw(target_temperature);
gov_sys_pol_attr_rw(io_is_busy);
gov_sys_pol_attr_rw(up_threshold);
gov_sys_pol_attr_rw(sampling_down_factor);
gov_sys_pol_attr_rw(ignore_nice_load);
gov_sys_pol_attr_rw(powersave_bias);
gov_sys_pol_attr_rw(Temp_Pred0);
gov_sys_pol_attr_rw(Temp_Pred1);
gov_sys_pol_attr_rw(Temp_Pred2);
gov_sys_pol_attr_rw(Temp_Pred3);
gov_sys_pol_attr_rw(Temp_Pred4);
gov_sys_pol_attr_rw(Temp0);
gov_sys_pol_attr_rw(Temp1);
gov_sys_pol_attr_rw(Temp2);
gov_sys_pol_attr_rw(Temp3);
gov_sys_pol_attr_rw(Temp4);
gov_sys_pol_attr_rw(GPU_util);
gov_sys_pol_attr_rw(Big_util);
gov_sys_pol_attr_rw(Little_util);
gov_sys_pol_attr_rw(GPU_temp_pred);
gov_sys_pol_attr_ro(sampling_rate_min);
gov_sys_pol_attr_rw(gpu_cur);
gov_sys_pol_attr_rw(gpu_next);
gov_sys_pol_attr_rw(cpu_cur);
gov_sys_pol_attr_rw(cpu_next);

static struct attribute *dbs_attributes_gov_sys[] = {
	&sampling_rate_min_gov_sys.attr,
	&cpu_cur_gov_sys.attr,
	&cpu_next_gov_sys.attr,
	&gpu_cur_gov_sys.attr,
	&gpu_next_gov_sys.attr,
	&sampling_rate_gov_sys.attr,
	&up_threshold_gov_sys.attr,
	&sampling_down_factor_gov_sys.attr,
	&ignore_nice_load_gov_sys.attr,
	&powersave_bias_gov_sys.attr,
	&io_is_busy_gov_sys.attr,
	&Temp_Pred0_gov_sys.attr,
	&Temp_Pred1_gov_sys.attr,
	&Temp_Pred2_gov_sys.attr,
	&Temp_Pred3_gov_sys.attr,
	&Temp_Pred4_gov_sys.attr,
	&Temp0_gov_sys.attr,
	&Temp1_gov_sys.attr,
	&Temp2_gov_sys.attr,
	&Temp3_gov_sys.attr,
	&Temp4_gov_sys.attr,
	&GPU_util_gov_sys.attr,
	&Big_util_gov_sys.attr,
	&Little_util_gov_sys.attr,
	&GPU_temp_pred_gov_sys.attr,
	&algorithm_mode_gov_sys.attr,
	&current_algorithm_gov_sys.attr,
	&target_temperature_gov_sys.attr,
	NULL
};

static struct attribute_group od_attr_group_gov_sys = {
	.attrs = dbs_attributes_gov_sys,
	.name = "ondemand",
};

static struct attribute *dbs_attributes_gov_pol[] = {
	&sampling_rate_min_gov_pol.attr,
	&cpu_cur_gov_pol.attr,
	&gpu_cur_gov_pol.attr,
	&cpu_next_gov_pol.attr,
	&gpu_next_gov_pol.attr,
	&sampling_rate_gov_pol.attr,
	&up_threshold_gov_pol.attr,
	&sampling_down_factor_gov_pol.attr,
	&ignore_nice_load_gov_pol.attr,
	&powersave_bias_gov_pol.attr,
	&io_is_busy_gov_pol.attr,
	&Temp_Pred0_gov_pol.attr,
	&Temp_Pred1_gov_pol.attr,
	&Temp_Pred2_gov_pol.attr,
	&Temp_Pred3_gov_pol.attr,
	&Temp_Pred4_gov_pol.attr,
	&Temp0_gov_pol.attr,
	&Temp1_gov_pol.attr,
	&Temp2_gov_pol.attr,
	&Temp3_gov_pol.attr,
	&Temp4_gov_pol.attr,
	&GPU_util_gov_pol.attr,
	&Big_util_gov_pol.attr,
	&Little_util_gov_pol.attr,
	&GPU_temp_pred_gov_pol.attr,
	&algorithm_mode_gov_pol.attr,
	&current_algorithm_gov_pol.attr,
	&target_temperature_gov_pol.attr,
	NULL
};

static struct attribute_group od_attr_group_gov_pol = {
	.attrs = dbs_attributes_gov_pol,
	.name = "ondemand",
};

/************************** sysfs end ************************/

static int od_init(struct dbs_data *dbs_data)
{
	struct od_dbs_tuners *tuners;
	u64 idle_time;
	int cpu;

	tuners = kzalloc(sizeof(struct od_dbs_tuners), GFP_KERNEL);
	if (!tuners) {
		pr_err("%s: kzalloc failed\n", __func__);
		return -ENOMEM;
	}

	cpu = get_cpu();
	idle_time = get_cpu_idle_time_us(cpu, NULL);
	put_cpu();
	if (idle_time != -1ULL) {
		/* Idle micro accounting is supported. Use finer thresholds */
		tuners->up_threshold = MICRO_FREQUENCY_UP_THRESHOLD;
		tuners->adj_up_threshold = MICRO_FREQUENCY_UP_THRESHOLD -
			MICRO_FREQUENCY_DOWN_DIFFERENTIAL;
		/*
		 * In nohz/micro accounting case we set the minimum frequency
		 * not depending on HZ, but fixed (very low). The deferred
		 * timer might skip some samples if idle/sleeping as needed.
		 */
		dbs_data->min_sampling_rate = MICRO_FREQUENCY_MIN_SAMPLE_RATE;
	} else {
		tuners->up_threshold = DEF_FREQUENCY_UP_THRESHOLD;
		tuners->adj_up_threshold = DEF_FREQUENCY_UP_THRESHOLD -
			DEF_FREQUENCY_DOWN_DIFFERENTIAL;

		/* For correct statistics, we need 10 ticks for each measure */
		dbs_data->min_sampling_rate = MIN_SAMPLING_RATE_RATIO *
			jiffies_to_usecs(10);
	}

	tuners->sampling_down_factor = DEF_SAMPLING_DOWN_FACTOR;
	tuners->algorithm_mode = DEFAULT_ALGORITHM_MODE;
	tuners->current_algorithm = DEFAULT_ALGORITHM_MODE;
	tuners->target_temperature = 68;
	tuners->ignore_nice_load = 0;
	tuners->powersave_bias = default_powersave_bias;
	tuners->io_is_busy = should_io_be_busy();
	tuners->Temp_Pred0 = 33;
	tuners->Temp_Pred1 = 43;
	tuners->Temp_Pred2 = 53;
	tuners->Temp_Pred3 = 63;
	tuners->Temp_Pred4 = 73;
	tuners->Temp0 = 0;
	tuners->Temp1 = 0;
	tuners->Temp2 = 0;
	tuners->Temp3 = 0;
	tuners->Temp4 = 0;
	tuners->GPU_util = 0;
	tuners->Big_util = 0;
	tuners->Little_util = 0;
	tuners->GPU_temp_pred = 133;
	tuners->gpu_cur = 0;
	tuners->cpu_cur = 0;
	tuners->gpu_next = 0;
	tuners->cpu_next = 0;
	dbs_data->tuners = tuners;
	mutex_init(&dbs_data->mutex);
	return 0;
}

static void od_exit(struct dbs_data *dbs_data)
{
	kfree(dbs_data->tuners);
}

define_get_cpu_dbs_routines(od_cpu_dbs_info);

static struct od_ops od_ops = {
	.powersave_bias_init_cpu = ondemand_powersave_bias_init_cpu,
	.powersave_bias_target = generic_powersave_bias_target,
	.freq_increase = dbs_freq_increase,
};

static struct common_dbs_data od_dbs_cdata = {
	.governor = GOV_ONDEMAND,
	.attr_group_gov_sys = &od_attr_group_gov_sys,
	.attr_group_gov_pol = &od_attr_group_gov_pol,
	.get_cpu_cdbs = get_cpu_cdbs,
	.get_cpu_dbs_info_s = get_cpu_dbs_info_s,
	.gov_dbs_timer = od_dbs_timer,
	.gov_check_cpu = od_check_cpu,
	.gov_ops = &od_ops,
	.init = od_init,
	.exit = od_exit,
};

static void od_set_powersave_bias(unsigned int powersave_bias)
{
	struct cpufreq_policy *policy;
	struct dbs_data *dbs_data;
	struct od_dbs_tuners *od_tuners;
	unsigned int cpu;
	cpumask_t done;

	default_powersave_bias = powersave_bias;
	cpumask_clear(&done);

	get_online_cpus();
	for_each_online_cpu(cpu) {
		if (cpumask_test_cpu(cpu, &done))
			continue;

		policy = per_cpu(od_cpu_dbs_info, cpu).cdbs.cur_policy;
		if (!policy)
			continue;

		cpumask_or(&done, &done, policy->cpus);

		if (policy->governor != &cpufreq_gov_ondemand)
			continue;

		dbs_data = policy->governor_data;
		od_tuners = dbs_data->tuners;
		od_tuners->powersave_bias = default_powersave_bias;
	}
	put_online_cpus();
}

	void od_register_powersave_bias_handler(unsigned int (*f)
			(struct cpufreq_policy *, unsigned int, unsigned int),
			unsigned int powersave_bias)
{
	od_ops.powersave_bias_target = f;
	od_set_powersave_bias(powersave_bias);
}
EXPORT_SYMBOL_GPL(od_register_powersave_bias_handler);

void od_unregister_powersave_bias_handler(void)
{
	od_ops.powersave_bias_target = generic_powersave_bias_target;
	od_set_powersave_bias(0);
}
EXPORT_SYMBOL_GPL(od_unregister_powersave_bias_handler);

static int od_cpufreq_governor_dbs(struct cpufreq_policy *policy,
		unsigned int event)
{
	return cpufreq_governor_dbs(policy, &od_dbs_cdata, event);
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMAND
static
#endif
struct cpufreq_governor cpufreq_gov_ondemand = {
	.name			= "ondemand",
	.governor		= od_cpufreq_governor_dbs,
	.max_transition_latency	= TRANSITION_LATENCY_LIMIT,
	.owner			= THIS_MODULE,
};

static int __init cpufreq_gov_dbs_init(void)
{
	return cpufreq_register_governor(&cpufreq_gov_ondemand);
}

static void __exit cpufreq_gov_dbs_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_ondemand);
}

MODULE_AUTHOR("Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>");
MODULE_AUTHOR("Alexey Starikovskiy <alexey.y.starikovskiy@intel.com>");
MODULE_DESCRIPTION("'cpufreq_ondemand' - A dynamic cpufreq governor for "
		"Low Latency Frequency Transition capable processors");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMAND
fs_initcall(cpufreq_gov_dbs_init);
#else
module_init(cpufreq_gov_dbs_init);
#endif
module_exit(cpufreq_gov_dbs_exit);
