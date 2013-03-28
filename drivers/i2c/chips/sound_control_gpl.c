/*
 * Author: Paul Reioux aka Faux123 <reioux@gmail.com>
 *
 * TPA2051D3 sound control module
 * Copyright 2013 Paul Reioux
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/kallsyms.h>

#include <linux/mutex.h>

static DEFINE_MUTEX(snd_hax_mutex);

#define MAX_CMD_SZ	7
extern char *htc_speaker_vol_control;
extern char *htc_headset_vol_control;
extern char *htc_ring_vol_control;
extern char *htc_handset_vol_control;
extern char *htc_lineout_vol_control;
#ifdef CONFIG_SND_CONTROL_HAS_BEATS
extern char *htc_beats_on_vol_control;
extern char *htc_beats_off_vol_control;
#endif

#define SOUND_CONTROL_MAJOR_VERSION	2
#define SOUND_CONTROL_MINOR_VERSION	1

#ifdef CONFIG_SND_CONTROL_HAS_BEATS
static ssize_t beats_on_gain_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d %d %d %d %d %d %d",
			htc_beats_on_vol_control[0],
			htc_beats_on_vol_control[1],
			htc_beats_on_vol_control[2],
			htc_beats_on_vol_control[3],
			htc_beats_on_vol_control[4],
			htc_beats_on_vol_control[5],
			htc_beats_on_vol_control[6]);
}

static ssize_t beats_on_gain_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int tmpIntArr[MAX_CMD_SZ];
	int i;

	sscanf(buf, "%d %d %d %d %d %d %d",
		&tmpIntArr[0],
		&tmpIntArr[1],
		&tmpIntArr[2],
		&tmpIntArr[3],
		&tmpIntArr[4],
		&tmpIntArr[5],
		&tmpIntArr[6]);
	mutex_lock(&snd_hax_mutex);
	for (i=0; i<MAX_CMD_SZ; i++) {
		htc_beats_on_vol_control[i] = (char)tmpIntArr[i];
	}
	mutex_unlock(&snd_hax_mutex);
	return (count);
}

static ssize_t beats_off_gain_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d %d %d %d %d %d %d",
			htc_beats_off_vol_control[0],
			htc_beats_off_vol_control[1],
			htc_beats_off_vol_control[2],
			htc_beats_off_vol_control[3],
			htc_beats_off_vol_control[4],
			htc_beats_off_vol_control[5],
			htc_beats_off_vol_control[6]);
}

static ssize_t beats_off_gain_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int tmpIntArr[MAX_CMD_SZ];
	int i;

	sscanf(buf, "%d %d %d %d %d %d %d",
		&tmpIntArr[0],
		&tmpIntArr[1],
		&tmpIntArr[2],
		&tmpIntArr[3],
		&tmpIntArr[4],
		&tmpIntArr[5],
		&tmpIntArr[6]);
	mutex_lock(&snd_hax_mutex);
	for (i=0; i<MAX_CMD_SZ; i++) {
		htc_beats_off_vol_control[i] = (char)tmpIntArr[i];
	}
	mutex_unlock(&snd_hax_mutex);
	return (count);
}
#endif

static ssize_t speaker_gain_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d %d %d %d %d %d %d",
			htc_speaker_vol_control[0],
			htc_speaker_vol_control[1],
			htc_speaker_vol_control[2],
			htc_speaker_vol_control[3],
			htc_speaker_vol_control[4],
			htc_speaker_vol_control[5],
			htc_speaker_vol_control[6]);
}

static ssize_t speaker_gain_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int tmpIntArr[MAX_CMD_SZ];
	int i;

	sscanf(buf, "%d %d %d %d %d %d %d",
		&tmpIntArr[0],
		&tmpIntArr[1],
		&tmpIntArr[2],
		&tmpIntArr[3],
		&tmpIntArr[4],
		&tmpIntArr[5],
		&tmpIntArr[6]);
	mutex_lock(&snd_hax_mutex);
	for (i=0; i<MAX_CMD_SZ; i++) {
		htc_speaker_vol_control[i] = (char)tmpIntArr[i];
	}
	mutex_unlock(&snd_hax_mutex);
	return (count);
}

static ssize_t headphone_gain_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d %d %d %d %d %d %d",
			htc_headset_vol_control[0],
			htc_headset_vol_control[1],
			htc_headset_vol_control[2],
			htc_headset_vol_control[3],
			htc_headset_vol_control[4],
			htc_headset_vol_control[5],
			htc_headset_vol_control[6]);
}

static ssize_t headphone_gain_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int tmpIntArr[MAX_CMD_SZ];
	int i;

	sscanf(buf, "%d %d %d %d %d %d %d",
		&tmpIntArr[0],
		&tmpIntArr[1],
		&tmpIntArr[2],
		&tmpIntArr[3],
		&tmpIntArr[4],
		&tmpIntArr[5],
		&tmpIntArr[6]);
	mutex_lock(&snd_hax_mutex);
	for (i=0; i<MAX_CMD_SZ; i++) {
		htc_headset_vol_control[i] = (char)tmpIntArr[i];
	}
	mutex_unlock(&snd_hax_mutex);
	return count;
}

static ssize_t handset_gain_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d %d %d %d %d %d %d",
			htc_handset_vol_control[0],
			htc_handset_vol_control[1],
			htc_handset_vol_control[2],
			htc_handset_vol_control[3],
			htc_handset_vol_control[4],
			htc_handset_vol_control[5],
			htc_handset_vol_control[6]);
}

static ssize_t handset_gain_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int tmpIntArr[MAX_CMD_SZ];
	int i;

	sscanf(buf, "%d %d %d %d %d %d %d",
		&tmpIntArr[0],
		&tmpIntArr[1],
		&tmpIntArr[2],
		&tmpIntArr[3],
		&tmpIntArr[4],
		&tmpIntArr[5],
		&tmpIntArr[6]);
	mutex_lock(&snd_hax_mutex);
	for (i=0; i<MAX_CMD_SZ; i++) {
		htc_handset_vol_control[i] = (char)tmpIntArr[i];
	}
	mutex_unlock(&snd_hax_mutex);
	return count;
}

static ssize_t ring_gain_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
        return sprintf(buf, "%d %d %d %d %d %d %d",
                        htc_ring_vol_control[0],
                        htc_ring_vol_control[1],
                        htc_ring_vol_control[2],
                        htc_ring_vol_control[3],
                        htc_ring_vol_control[4],
                        htc_ring_vol_control[5],
                        htc_ring_vol_control[6]);
}

static ssize_t ring_gain_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int tmpIntArr[MAX_CMD_SZ];
	int i;

	sscanf(buf, "%d %d %d %d %d %d %d",
		&tmpIntArr[0],
		&tmpIntArr[1],
		&tmpIntArr[2],
		&tmpIntArr[3],
		&tmpIntArr[4],
		&tmpIntArr[5],
		&tmpIntArr[6]);
	mutex_lock(&snd_hax_mutex);
	for (i=0; i<MAX_CMD_SZ; i++) {
		htc_ring_vol_control[i] = (char)tmpIntArr[i];
	}
	mutex_unlock(&snd_hax_mutex);
	return count;
}

static ssize_t lineout_gain_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
        return sprintf(buf, "%d %d %d %d %d %d %d",
                        htc_lineout_vol_control[0],
                        htc_lineout_vol_control[1],
                        htc_lineout_vol_control[2],
                        htc_lineout_vol_control[3],
                        htc_lineout_vol_control[4],
                        htc_lineout_vol_control[5],
                        htc_lineout_vol_control[6]);
}

static ssize_t lineout_gain_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int tmpIntArr[MAX_CMD_SZ];
	int i;

	sscanf(buf, "%d %d %d %d %d %d %d",
		&tmpIntArr[0],
		&tmpIntArr[1],
		&tmpIntArr[2],
		&tmpIntArr[3],
		&tmpIntArr[4],
		&tmpIntArr[5],
		&tmpIntArr[6]);
	mutex_lock(&snd_hax_mutex);
	for (i=0; i<MAX_CMD_SZ; i++) {
		htc_lineout_vol_control[i] = (char)tmpIntArr[i];
	}
	mutex_unlock(&snd_hax_mutex);
	return count;
}

static ssize_t sound_control_version_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "version: %u.%u\n",
			SOUND_CONTROL_MAJOR_VERSION,
			SOUND_CONTROL_MINOR_VERSION);
}

#ifdef CONFIG_SND_CONTROL_HAS_BEATS
static struct kobj_attribute beats_on_gain_attribute =
	__ATTR(gpl_beats_on_gain,
		0666,
		beats_on_gain_show,
		beats_on_gain_store);

static struct kobj_attribute beats_off_gain_attribute =
	__ATTR(gpl_beats_off_gain,
		0666,
		beats_off_gain_show,
		beats_off_gain_store);
#endif

static struct kobj_attribute speaker_gain_attribute =
	__ATTR(gpl_speaker_gain,
		0666,
		speaker_gain_show,
		speaker_gain_store);

static struct kobj_attribute headphone_gain_attribute = 
	__ATTR(gpl_headphone_gain,
		0666,
		headphone_gain_show,
		headphone_gain_store);

static struct kobj_attribute handset_gain_attribute =
	__ATTR(gpl_handset_gain,
		0666,
		handset_gain_show,
		handset_gain_store);

static struct kobj_attribute ring_gain_attribute = 
	__ATTR(gpl_ring_gain,
		0666,
		ring_gain_show,
		ring_gain_store);

static struct kobj_attribute lineout_gain_attribute = 
	__ATTR(gpl_lineout_gain,
		0666,
		lineout_gain_show,
		lineout_gain_store);

static struct kobj_attribute sound_control_version_attribute = 
	__ATTR(gpl_sound_control_version,
		0444,
		sound_control_version_show, NULL);

static struct attribute *sound_control_attrs[] =
	{
#ifdef CONFIG_SND_CONTROL_HAS_BEATS
		&beats_on_gain_attribute.attr,
		&beats_off_gain_attribute.attr,
#endif
		&speaker_gain_attribute.attr,
		&headphone_gain_attribute.attr,
		&handset_gain_attribute.attr,
		&ring_gain_attribute.attr,
		&lineout_gain_attribute.attr,
		&sound_control_version_attribute.attr,
		NULL,
	};

static struct attribute_group sound_control_attr_group =
	{
		.attrs = sound_control_attrs,
	};

static struct kobject *sound_control_kobj;

static int sound_control_init(void)
{
	int sysfs_result;

	if (htc_headset_vol_control == NULL) {
		pr_err("%s sound_controls_ptr is NULL!\n", __FUNCTION__);
		return -1;
	}

	sound_control_kobj =
		kobject_create_and_add("sound_control_tpa", kernel_kobj);

	if (!sound_control_kobj) {
		pr_err("%s sound_control_kobj create failed!\n",
			__FUNCTION__);
		return -ENOMEM;
        }

	sysfs_result = sysfs_create_group(sound_control_kobj,
			&sound_control_attr_group);

	if (sysfs_result) {
		pr_info("%s sysfs create failed!\n", __FUNCTION__);
		kobject_put(sound_control_kobj);
	}
	return sysfs_result;
}

static void sound_control_exit(void)
{
	if (sound_control_kobj != NULL)
		kobject_put(sound_control_kobj);
}

module_init(sound_control_init);
module_exit(sound_control_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Paul Reioux <reioux@gmail.com>");
MODULE_DESCRIPTION("Sound Control Module GPL Edition");

