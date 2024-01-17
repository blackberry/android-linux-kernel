/*
 * Copyright 2011-2014 by Emese Revfy <re.emese@gmail.com>
 * Licensed under the GPL v2, or (at your option) v3
 *
 * Homepage:
 * http://www.grsecurity.net/~ephox/overflow_plugin/
 *
 * Documentation:
 * http://forums.grsecurity.net/viewtopic.php?f=7&t=3043
 *
 * This plugin recomputes expressions of function arguments marked by a size_overflow attribute
 * with double integer precision (DImode/TImode for 32/64 bit integer types).
 * The recomputed argument is checked against TYPE_MAX and an event is logged on overflow and the triggering process is killed.
 *
 * Usage:
 * $ make
 * $ make run
 */

#include "gcc-common.h"
#include "size_overflow.h"

int plugin_is_GPL_compatible;

tree report_size_overflow_decl;

tree size_overflow_type_HI;
tree size_overflow_type_SI;
tree size_overflow_type_DI;
tree size_overflow_type_TI;

static struct plugin_info size_overflow_plugin_info = {
	.version	= "20140725",
	.help		= "no-size-overflow\tturn off size overflow checking\n",
};

static tree handle_size_overflow_attribute(tree *node, tree __unused name, tree args, int __unused flags, bool *no_add_attrs)
{
	unsigned int arg_count;
	enum tree_code code = TREE_CODE(*node);

	switch (code) {
	case FUNCTION_DECL:
		arg_count = type_num_arguments(TREE_TYPE(*node));
		break;
	case FUNCTION_TYPE:
	case METHOD_TYPE:
		arg_count = type_num_arguments(*node);
		break;
	default:
		*no_add_attrs = true;
		error("%s: %qE attribute only applies to functions", __func__, name);
		return NULL_TREE;
	}

	for (; args; args = TREE_CHAIN(args)) {
		tree position = TREE_VALUE(args);
		if (TREE_CODE(position) != INTEGER_CST || TREE_INT_CST_LOW(position) > arg_count ) {
			error("%s: parameter %u is outside range.", __func__, (unsigned int)TREE_INT_CST_LOW(position));
			*no_add_attrs = true;
		}
	}
	return NULL_TREE;
}

static tree handle_intentional_overflow_attribute(tree *node, tree __unused name, tree args, int __unused flags, bool *no_add_attrs)
{
	unsigned int arg_count;
	enum tree_code code = TREE_CODE(*node);

	switch (code) {
	case FUNCTION_DECL:
		arg_count = type_num_arguments(TREE_TYPE(*node));
		break;
	case FUNCTION_TYPE:
	case METHOD_TYPE:
		arg_count = type_num_arguments(*node);
		break;
	case FIELD_DECL:
		return NULL_TREE;
	default:
		*no_add_attrs = true;
		error("%qE attribute only applies to functions", name);
		return NULL_TREE;
	}

	if (TREE_INT_CST_HIGH(TREE_VALUE(args)) != 0)
		return NULL_TREE;

	for (; args; args = TREE_CHAIN(args)) {
		tree position = TREE_VALUE(args);
		if (TREE_CODE(position) != INTEGER_CST || TREE_INT_CST_LOW(position) > arg_count ) {
			error("%s: parameter %u is outside range.", __func__, (unsigned int)TREE_INT_CST_LOW(position));
			*no_add_attrs = true;
		}
	}
	return NULL_TREE;
}

static struct attribute_spec size_overflow_attr = {
	.name				= "size_overflow",
	.min_length			= 1,
	.max_length			= -1,
	.decl_required			= true,
	.type_required			= false,
	.function_type_required		= false,
	.handler			= handle_size_overflow_attribute,
#if BUILDING_GCC_VERSION >= 4007
	.affects_type_identity		= false
#endif
};

static struct attribute_spec intentional_overflow_attr = {
	.name				= "intentional_overflow",
	.min_length			= 1,
	.max_length			= -1,
	.decl_required			= true,
	.type_required			= false,
	.function_type_required		= false,
	.handler			= handle_intentional_overflow_attribute,
#if BUILDING_GCC_VERSION >= 4007
	.affects_type_identity		= false
#endif
};

static void register_attributes(void __unused *event_data, void __unused *data)
{
	register_attribute(&size_overflow_attr);
	register_attribute(&intentional_overflow_attr);
}

static tree create_typedef(tree type, const char* ident)
{
	tree new_type, decl;

	new_type = build_variant_type_copy(type);
	decl = build_decl(BUILTINS_LOCATION, TYPE_DECL, get_identifier(ident), new_type);
	DECL_ORIGINAL_TYPE(decl) = type;
	TYPE_NAME(new_type) = decl;
	return new_type;
}

// Create the noreturn report_size_overflow() function decl.
static void size_overflow_start_unit(void __unused *gcc_data, void __unused *user_data)
{
	tree const_char_ptr_type_node;
	tree fntype;

	const_char_ptr_type_node = build_pointer_type(build_type_variant(char_type_node, 1, 0));

	size_overflow_type_HI = create_typedef(intHI_type_node, "size_overflow_type_HI");
	size_overflow_type_SI = create_typedef(intSI_type_node, "size_overflow_type_SI");
	size_overflow_type_DI = create_typedef(intDI_type_node, "size_overflow_type_DI");
	size_overflow_type_TI = create_typedef(intTI_type_node, "size_overflow_type_TI");

	// void report_size_overflow(const char *loc_file, unsigned int loc_line, const char *current_func, const char *ssa_var)
	fntype = build_function_type_list(void_type_node,
					  const_char_ptr_type_node,
					  unsigned_type_node,
					  const_char_ptr_type_node,
					  const_char_ptr_type_node,
					  NULL_TREE);
	report_size_overflow_decl = build_fn_decl("report_size_overflow", fntype);

	DECL_ASSEMBLER_NAME(report_size_overflow_decl);
	TREE_PUBLIC(report_size_overflow_decl) = 1;
	DECL_EXTERNAL(report_size_overflow_decl) = 1;
	DECL_ARTIFICIAL(report_size_overflow_decl) = 1;
	TREE_THIS_VOLATILE(report_size_overflow_decl) = 1;
}


extern struct gimple_opt_pass pass_dce;

static struct opt_pass *make_dce_pass(void)
{
#if BUILDING_GCC_VERSION >= 4009
	return make_pass_dce(g);
#else
	return &pass_dce.pass;
#endif
}


int plugin_init(struct plugin_name_args *plugin_info, struct plugin_gcc_version *version)
{
	int i;
	const char * const plugin_name = plugin_info->base_name;
	const int argc = plugin_info->argc;
	const struct plugin_argument * const argv = plugin_info->argv;
	bool enable = true;
	struct register_pass_info insert_size_overflow_asm_pass_info;
	struct register_pass_info __unused dump_before_pass_info;
	struct register_pass_info __unused dump_after_pass_info;
	struct register_pass_info insert_size_overflow_check_info;
	struct register_pass_info dce_pass_info;
	static const struct ggc_root_tab gt_ggc_r_gt_size_overflow[] = {
		{
			.base = &report_size_overflow_decl,
			.nelt = 1,
			.stride = sizeof(report_size_overflow_decl),
			.cb = &gt_ggc_mx_tree_node,
			.pchw = &gt_pch_nx_tree_node
		},
		LAST_GGC_ROOT_TAB
	};

	insert_size_overflow_asm_pass_info.pass				= make_insert_size_overflow_asm_pass();
	insert_size_overflow_asm_pass_info.reference_pass_name		= "ssa";
	insert_size_overflow_asm_pass_info.ref_pass_instance_number	= 1;
	insert_size_overflow_asm_pass_info.pos_op			= PASS_POS_INSERT_AFTER;

	dump_before_pass_info.pass			= make_dump_pass();
	dump_before_pass_info.reference_pass_name	= "increase_alignment";
	dump_before_pass_info.ref_pass_instance_number	= 1;
	dump_before_pass_info.pos_op			= PASS_POS_INSERT_BEFORE;

	insert_size_overflow_check_info.pass			= make_insert_size_overflow_check();
	insert_size_overflow_check_info.reference_pass_name	= "increase_alignment";
	insert_size_overflow_check_info.ref_pass_instance_number	= 1;
	insert_size_overflow_check_info.pos_op			= PASS_POS_INSERT_BEFORE;

	dump_after_pass_info.pass			= make_dump_pass();
	dump_after_pass_info.reference_pass_name	= "increase_alignment";
	dump_after_pass_info.ref_pass_instance_number	= 1;
	dump_after_pass_info.pos_op			= PASS_POS_INSERT_BEFORE;

	dce_pass_info.pass				= make_dce_pass();
	dce_pass_info.reference_pass_name		= "vrp";
	dce_pass_info.ref_pass_instance_number	= 1;
	dce_pass_info.pos_op			= PASS_POS_INSERT_AFTER;

	if (!plugin_default_version_check(version, &gcc_version)) {
		error(G_("incompatible gcc/plugin versions"));
		return 1;
	}

	for (i = 0; i < argc; ++i) {
		if (!strcmp(argv[i].key, "no-size-overflow")) {
			enable = false;
			continue;
		}
		error(G_("unkown option '-fplugin-arg-%s-%s'"), plugin_name, argv[i].key);
	}

	register_callback(plugin_name, PLUGIN_INFO, NULL, &size_overflow_plugin_info);
	if (enable) {
		register_callback(plugin_name, PLUGIN_START_UNIT, &size_overflow_start_unit, NULL);
		register_callback(plugin_name, PLUGIN_REGISTER_GGC_ROOTS, NULL, (void *)&gt_ggc_r_gt_size_overflow);
		register_callback(plugin_name, PLUGIN_PASS_MANAGER_SETUP, NULL, &insert_size_overflow_asm_pass_info);
//		register_callback(plugin_name, PLUGIN_PASS_MANAGER_SETUP, NULL, &dump_before_pass_info);
		register_callback(plugin_name, PLUGIN_PASS_MANAGER_SETUP, NULL, &insert_size_overflow_check_info);
//		register_callback(plugin_name, PLUGIN_PASS_MANAGER_SETUP, NULL, &dump_after_pass_info);
		register_callback(plugin_name, PLUGIN_PASS_MANAGER_SETUP, NULL, &dce_pass_info);
	}
	register_callback(plugin_name, PLUGIN_ATTRIBUTES, register_attributes, NULL);

	return 0;
}
