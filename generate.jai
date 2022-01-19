TYPES_TO_OMIT :: string.[
    "D3DCOMPILE_OPTIMIZATION_LEVEL1", //For now, until we can handle "(1 << x)" macro values and can generate all the values at once.
    "D3DCreateFunctionLinkingGraph", // Because it pulls in a huge amount of other types
];

SYSTEM_TYPES_TO_INCLUDE :: string.[
    "ID3D10Blob",
    "ID3D11Linker",
    "ID3D11Module",
    "ID3D11ModuleInstance",
    "ID3DBlob",
    "ID3DInclude",
    "_D3D_INCLUDE_TYPE",
    "_D3D_SHADER_MACRO",
];

#run {
    set_build_options_dc(.{do_output=false});

    write_file := true;
    user_args := compiler_get_command_line_arguments();
    for user_args {
        if it == "-no_write"    write_file = false;
    }
    generate_bindings(write_file = write_file);
}

// Since compile-time debugging is still tricky, it can sometimes be useful to compile this script into a binary and run that:

// #run {
//     if get_current_workspace() != 2   return;

//     set_build_options_dc(.{do_output=false});
//     options := get_build_options();

//     w := compiler_create_workspace("Main program");
//     options.output_type     = .EXECUTABLE;
//     options.output_executable_name = "generate";
//     set_build_options(options, w);
    
//     add_build_file("generate.jai", w);
// }

// main :: () {
//     // set_working_directory(path_strip_filename(get_path_of_running_executable()));
//     generate_bindings();
// }


generate_bindings :: (write_file := true) {
    output_filename: string;
    opts: Generate_Bindings_Options;
    {
        using opts;

        #if OS == .WINDOWS {
            output_filename = "module.jai";
        } else {
            assert(false);
        }
        array_add(*libpaths, "bin/win");
        array_add(*libnames, "box2djai");

        array_add(*system_include_paths, GENERATOR_DEFAULT_SYSTEM_INCLUDE_PATH);

        files := file_list(tprint("%box2d/wrapper", #filepath), true);
        for file : files {
            if !ends_with(file, ".h") {
                continue;
            }

            print("HEADER FILE: %\n", file);

            array_add(*source_files, file);
        }

        array_add(*extra_clang_arguments, "-x", "c++");
    }

    bindings_builder, success := generate_bindings(opts);
    if !success return;
    defer free_buffers(*bindings_builder);
    show_generator_results();
    
    if write_file {
        success = write_entire_file(output_filename, bindings_builder);
        if !success {
            log_error("Could not write generated output to \"%\"\n", output_filename);
            return;
        }
        log("\nOK! generated '%'\n", output_filename);
    }
}

#scope_file

#import "Basic";
#import "Bindings_Generator";
#import "BuildCpp";
#import "Compiler";
#import "File";
#import "String";
#import "System";
#import "Windows_Resources";
#import "File_Utilities";