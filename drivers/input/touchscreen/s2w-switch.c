/*
 * Author: Jean-Pierre Rasquin <yank555.lu@gmail.com>
 *
 * Inspired by the work of Chad Froebel <chadfroebel@gmail.com>
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

#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/s2w-switch.h>

/* sysfs interface */
static ssize_t s2w_switch_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
return sprintf(buf, "%d\n", s2w_switch);
}

static ssize_t s2w_switch_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
sscanf(buf, "%du", &s2w_switch);
return count;
}

static struct kobj_attribute s2w_switch_attribute =
__ATTR(s2w_switch, 0666, s2w_switch_show, s2w_switch_store);

static struct attribute *attrs[] = {
&s2w_switch_attribute.attr,
NULL,
};

static struct attribute_group attr_group = {
.attrs = attrs,
};

static struct kobject *s2w_switch_kobj;

int s2w_switch_init(void)
{
	int retval;

        s2w_switch_kobj = kobject_create_and_add("sweep2wake", kernel_kobj);
        if (!s2w_switch_kobj) {
                return -ENOMEM;
        }
        retval = sysfs_create_group(s2w_switch_kobj, &attr_group);
        if (retval)
                kobject_put(s2w_switch_kobj);
        return retval;
}
/* end sysfs interface */

void s2w_switch_exit(void)
{
	kobject_put(s2w_switch_kobj);
}

module_init(s2w_switch_init);
module_exit(s2w_switch_exit);
