#run generate();

FunctionData :: struct {
    type: string;
    name: string;
    // type, name
    parameters: [..] [2] string;
}

generate :: () {
    set_build_options_dc(.{do_output = false});

    make_directory_if_it_does_not_exist(tprint("%box2d/build", #filepath));
    make_directory_if_it_does_not_exist(tprint("%assertgen/build", #filepath));

    print("Generating dll and lib...");

    generated_dll := os_run_command(tprint("%box2d/build.bat", #filepath), working_directory=tprint("%box2d/build", #filepath));
    assert(generated_dll, "Could not generate dll.");

    file_delete(tprint("%bin/win/box2djai.dll", #filepath));
    file_delete(tprint("%bin/win/box2djai.lib", #filepath));

    assert(file_move(tprint("%box2d/build/RelWithDebInfo/box2djai.dll", #filepath), tprint("%bin/win/box2djai.dll", #filepath)), "Could not move dll.");
    assert(file_move(tprint("%box2d/build/RelWithDebInfo/box2djai.lib", #filepath), tprint("%bin/win/box2djai.lib", #filepath)), "Could not move lib.");

    file_builder: String_Builder;

    files := file_list(tprint("%box2d/wrapper", #filepath), true);
    for file: files {
        if !ends_with(file, ".h") {
            continue;
        }

        // print("HEADER FILE: %\n", file);

        contents, read := read_entire_file(file);
        if !read {
            print("File error: %\n", file);
            assert(false, "Could not read file.");
        }

        append(*file_builder, contents);
        append(*file_builder, "\n");
    }

    file_contents := builder_to_string(*file_builder);
    lines := split(file_contents, "\n");

    function_data: [..] FunctionData;
    current_function_data: *FunctionData;

    reading_function_type := false;
    reading_function_name := false;
    reading_parameter_name := false;
    reading_parameter_type := false;
    current_builder: String_Builder;

    character := 0;

    i := 0;
    while i < lines.count {
        defer i += 1;
        character = 0;

        print("%\n", lines[i]);

        line := lines[i];
        line = replace(line, "const", "");

        if current_function_data && starts_with(line, "EXPORT") {
            assert(false, "Found an EXPORT line while expecting a continuation.");
        }

        if starts_with(line, "EXPORT") {
            data: FunctionData;
            array_add(*function_data, data);
            current_function_data = *function_data[function_data.count - 1];

            reading_function_type = true;
            reading_function_name = false;
            reading_parameter_type = false;
            reading_parameter_name = false;
            character = "EXPORT".count + 1; // 7;

            reset(*current_builder);
        }

        if !current_function_data {
            continue;
        }

        while true {
            defer character += 1;

            if character >= line.count || line[character] == #char "\r" || line[character] == #char "\n" || line[character] == #char " " || line[character] == #char "," || line[character] == #char "(" || line[character] == #char ")" || line[character] == #char ";" {
                if builder_string_length(*current_builder) > 0 {
                    if reading_function_type {
                        current_function_data.type = builder_to_string(*current_builder);
                        reset(*current_builder);

                        reading_function_type = false;
                        reading_function_name = true;
                        reading_parameter_type = false;
                        reading_parameter_name = false;
                    } else if reading_function_name {
                        current_function_data.name = builder_to_string(*current_builder);
                        reset(*current_builder);

                        reading_function_type = false;
                        reading_function_name = false;
                        reading_parameter_type = true;
                        reading_parameter_name = false;
                    } else if reading_parameter_type {
                        parameters: [2] string;
                        parameters[0] = builder_to_string(*current_builder);
                        reset(*current_builder);

                        array_add(*current_function_data.parameters, parameters);

                        reading_function_type = false;
                        reading_function_name = false;
                        reading_parameter_type = false;
                        reading_parameter_name = true;
                    } else if reading_parameter_name {
                        current_function_data.parameters[current_function_data.parameters.count - 1][1] = builder_to_string(*current_builder);
                        reset(*current_builder);

                        reading_function_type = false;
                        reading_function_name = false;
                        reading_parameter_type = true;
                        reading_parameter_name = false;
                    }
                }

                if character >= line.count {
                    break;
                } else if line[character] == #char ";" {
                    current_function_data = null;
                    break;
                }
            } else {
                append(*current_builder, line[character]);
            }
        }
    }

    print("Preparing to write file...\n");

    reset(*file_builder);

    REPLACEMENT_TYPES :: [2] string.[
        string.["int32_t", "s32"],
        string.["b2Vec2", "Vector2"],
    ];

    // fix pointers
    for *data: function_data {
        for replacement_pair: REPLACEMENT_TYPES {
            if starts_with(data.type, replacement_pair[0]) {
                data.type = replace(data.type, replacement_pair[0], replacement_pair[1]);
            }
        }

        if ends_with(data.type, "*") {
            i := data.type.count - 1;
            while i > 0 {
                data.type.data[i] = data.type.data[i - 1];
                i -= 1;
            }

            data.type.data[0] = #char "*";
        }

        for *parameter: data.parameters {
            for replacement_pair: REPLACEMENT_TYPES {
                if starts_with((<< parameter)[0], replacement_pair[0]) {
                    (<< parameter)[0] = replace((<< parameter)[0], replacement_pair[0], replacement_pair[1]);
                }
            }

            if ends_with((<< parameter)[0], "*") {
                i := (<< parameter)[0].count - 1;
                while i > 0 {
                    (<< parameter)[0].data[i] = (<< parameter)[0].data[i - 1];
                    i -= 1;
                }

                (<< parameter)[0].data[0] = #char "*";
            }
        }

        // print the function
        append(*file_builder, data.name);
        append(*file_builder, " :: (");
    
        for i: 0..data.parameters.count - 1 {
            parameters := data.parameters[i];

            append(*file_builder, parameters[1]);
            append(*file_builder, ": ");
            append(*file_builder, parameters[0]);

            if i < data.parameters.count - 1 {
                append(*file_builder, ", ");
            }
        }
        append(*file_builder, ") -> ");
        append(*file_builder, data.type);
        append(*file_builder, " #foreign box2d;\n");
    }

    append(*file_builder, "\n#scope_file\n\nbox2d :: #foreign_library \"bin/win/box2djai\";\n");

    write_entire_file("functions.jai", builder_to_string(*file_builder));

    found_types: [..] string;
    for data: function_data {
        {
            type := data.type;
            if starts_with(type, "*") {
                type = slice(type, 1, type.count - 1);
            }

            found := array_find(found_types, type);
            if !found {
                array_add(*found_types, type);
            }
        }

        for parameters: data.parameters {
            type := parameters[0];
            if starts_with(type, "*") {
                type = slice(type, 1, type.count - 1);
            }

            found := array_find(found_types, type);
            if !found {
                array_add(*found_types, type);
            }
        }
    }

    array_add(*found_types, "b2Mat22");
    array_add(*found_types, "b2Mat33");
    array_add(*found_types, "b2Sweep");
    array_add(*found_types, "b2ContactImpulse");
    array_add(*found_types, "b2BlockAllocator");
    array_add(*found_types, "b2Chunk");
    array_add(*found_types, "b2Block");
    array_add(*found_types, "b2FixtureProxy");
    array_add(*found_types, "b2StackAllocator");
    array_add(*found_types, "b2StackEntry");
    array_add(*found_types, "b2JointType");
    array_add(*found_types, "b2BroadPhase");
    array_add(*found_types, "b2DynamicTree");
    array_add(*found_types, "b2Pair");
    array_add(*found_types, "b2TreeNode");

    IGNORE_TYPES :: string.[
        "void",
        "s32",
        "Vector2",
        "float",
        "bool",
    ];

    IGNORE_FOR_ASSSERTS :: string.[
        "b2Chunk",
        "b2Block",
    ];

    reset(*file_builder);
    for type: found_types {
        if array_find(IGNORE_TYPES, type) {
            continue;
        }

        append(*file_builder, type);
        append(*file_builder, " :: struct {");
        append(*file_builder, "\n");
        append(*file_builder, "\n");
        append(*file_builder, "}");
        append(*file_builder, "\n");
        append(*file_builder, "\n");
    }

    append(*file_builder, "#scope_file\n\n#import \"Math\";\n\n");

    write_entire_file("newstructs.jai", builder_to_string(*file_builder));

    print("Creating assertgen.cpp file...\n");

    reset(*file_builder);
    append(*file_builder, #string DONE
#include <iostream>
#include <fstream>

#include "../box2d/lib.cpp"

int main() {
    std::ofstream assertFile;
    assertFile.open("../../asserts.jai");
    assertFile << "#run {\n";
    DONE);

    for type: found_types {
        if array_find(IGNORE_TYPES, type) {
            continue;
        }

        if array_find(IGNORE_FOR_ASSSERTS, type) {
            continue;
        }
        
        append(*file_builder, "    assertFile << \"    assert(size_of(");
        append(*file_builder, type);
        append(*file_builder, ") == \" << sizeof(");
        append(*file_builder, type);
        append(*file_builder, ")");
        append(*file_builder, " << \", \\\"");
        append(*file_builder, type);
        append(*file_builder, " must be of size \" << sizeof(");
        append(*file_builder, type);
        append(*file_builder, ") << \".\\\");\\n\";\n");
    }

    append(*file_builder, #string DONE
    assertFile << "}\n\n#scope_file\n\n#import \"Basic\";\n";
    assertFile.close();
    return 0;
}
    DONE);

    write_entire_file("assertgen/main.cpp", builder_to_string(*file_builder));

    print("Generating asserts.jai...\n");

    success, exit_code, output_string, error_string := os_run_command(tprint("%assertgen/generate_asserts.bat", #filepath), working_directory=tprint("%assertgen/build", #filepath));
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
#import "Process";