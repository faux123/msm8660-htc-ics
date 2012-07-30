/*
 * Author: Jean-Pierre Rasquin <yank555.lu@gmail.com>
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

/*
 * Possible values for "off_timer_multiplier" are :
 *
 *   0     - infinite
 *   1     - as requested by process (default)
 *   2-255 - requested by process multiplied by given value
 *
 */

#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/leds-pm8058-multiplier.h>

int off_timer_multiplier;

/* sysfs interface for "off_timer_multiplier" */
static ssize_t off_timer_multiplier_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{

	switch (off_timer_multiplier) {
		case OFF_TIMER_INFINITE:	return sprintf(buf, "Infinite\n");
		case OFF_TIMER_NORMAL:		return sprintf(buf, "As requested by process\n");
		default:			return sprintf(buf, "Process request x %d\n", off_timer_multiplier);
	}

}

static ssize_t off_timer_multiplier_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{

int new_off_timer_multiplier;

sscanf(buf, "%du", &new_off_timer_multiplier);

if (new_off_timer_multiplier >= OFF_TIMER_INFINITE && new_off_timer_multiplier <= OFF_TIMER_MAX) {

	/* update only if valid value provided */
	off_timer_multiplier = new_off_timer_multiplier;

}

return count;
}

static struct kobj_attribute off_timer_multiplier_attribute =
__ATTR(off_timer_multiplier, 0666, off_timer_multiplier_show, off_timer_multiplier_store);

static struct attribute *off_timer_multiplier_attrs[] = {
&off_timer_multiplier_attribute.attr,
NULL,
};

static struct attribute_group off_timer_multiplier_attr_group = {
.attrs = off_timer_multiplier_attrs,
};

/* Initialize fast charge sysfs folder */
static struct kobject *off_timer_multiplier_kobj;

int off_timer_multiplier_init(void)
{
	int off_timer_multiplier_retval;

	off_timer_multiplier = OFF_TIMER_NORMAL; /* Respect notification delay requested by process */

        off_timer_multiplier_kobj = kobject_create_and_add("notification_leds", kernel_kobj);
        if (!off_timer_multiplier_kobj) {
                return -ENOMEM;
        }
        off_timer_multiplier_retval = sysfs_create_group(off_timer_multiplier_kobj, &off_timer_multiplier_attr_group);
        if (off_timer_multiplier_retval)
                kobject_put(off_timer_multiplier_kobj);
        return (off_timer_multiplier_retval);
}
/* end sysfs interface */

void off_timer_multiplier_exit(void)
{
	kobject_put(off_timer_multiplier_kobj);
}

module_init(off_timer_multiplier_init);
module_exit(off_timer_multiplier_exit);
