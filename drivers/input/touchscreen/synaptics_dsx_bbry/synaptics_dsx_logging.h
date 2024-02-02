/*
 * Copyright (C) 2015 Blackberry
 * Copyright (C) 2010 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SYNAPTICS_LOGGING_H
#define SYNAPTICS_LOGGING_H

#include <linux/kernel.h>

#define synaptics_info(msg, ...) \
{ \
	pr_info("%s:" msg "\n", __func__, ##__VA_ARGS__); \
}

#define synaptics_warn(msg, ...) \
{ \
	pr_warn("%s:" msg "\n", __func__, ##__VA_ARGS__); \
}

#define synaptics_err(msg, ...) \
{ \
	pr_err("%s:" msg "\n", __func__, ##__VA_ARGS__); \
}

#endif /* SYNAPTICS_LOGGING_H */
