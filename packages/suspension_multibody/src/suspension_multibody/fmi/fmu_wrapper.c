/*
 * The FMI 2.0 Co-Simulation wrapper for an exported suspension_multibody run.
 *
 * D4's scope is what this file is: the FMU carries the *model* and its
 * *input/output variables*, and it does its own solving.  There is no Python in
 * the loop at run time -- `fmi2Instantiate` reads the two contract containers
 * that were shipped as resources, `fmi2DoStep` loads the native kernel and calls
 * `suspension_kernel_run` on them, and `fmi2GetReal` publishes the result blocks
 * the run produced.
 *
 * Why the kernel is loaded rather than linked: the FMU is portable across a
 * machine that has the built kernel and one that does not, and a co-simulation
 * import that failed at *load* time would be indistinguishable from a broken
 * archive.  `LoadLibrary` at instantiate time turns "the kernel is missing" into
 * a reported instance-level error with the path in it.
 *
 * The symbol set below is the full FMI 2.0 Co-Simulation surface.  The optional
 * parts (FMU state serialisation, directional derivatives) answer `fmi2Error`
 * rather than lying about support.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

/* --- FMI 2.0 types, spelled out so the file needs no FMI header ----------- */

typedef unsigned int fmi2ValueReference;
typedef double fmi2Real;
typedef int fmi2Integer;
typedef int fmi2Boolean;
typedef char fmi2Char;
typedef const fmi2Char* fmi2String;
typedef void* fmi2Component;
typedef void* fmi2ComponentEnvironment;
typedef void* fmi2FMUstate;

typedef enum {
    fmi2OK, fmi2Warning, fmi2Discard, fmi2Error, fmi2Fatal, fmi2Pending
} fmi2Status;

typedef enum {
    fmi2DoStepStatus, fmi2PendingStatus, fmi2LastSuccessfulTime, fmi2Terminated
} fmi2StatusKind;

typedef enum { fmi2ModelExchange, fmi2CoSimulation } fmi2Type;

/* The kernel's documented entry point.  The signature is the one in
 * `cpp/axle_dynamics/axle_kernel.hpp`: model payload, case payload, an output
 * buffer whose length is in/out, and an error buffer.  Status 0 is success and
 * 11 is "the buffer was too small, try again with the reported size". */
typedef int (*kernel_run_fn)(const unsigned char*, size_t, const unsigned char*,
                             size_t, unsigned char*, size_t*, char*, size_t);

#define MAX_PATH_LEN 4096
#define ERROR_CAPACITY 512
/*: Maximum number of real variables one instance may publish or consume.  It is
 *: a compile-time bound because the FMI interface is value references into a
 *: flat table, and a fixed table is what keeps this file allocation-free in the
 *: stepping path. */
#define MAX_VARIABLES 512

/*: Where one variable's number lives.  `input` names a scalar slot inside the
 *: case container's blob -- written at every sample of that slot, because FMI
 *: has no vector-valued real.  `output` names a block and column of the result
 *: container's blob, read at the sample the instance's clock is at.  The record
 *: is filled from `resources/bindings.txt`, which the exporter generates from
 *: the same bindings it declares in `modelDescription.xml`; nothing here
 *: restates a layout by hand. */
typedef struct {
    fmi2Real value;
    int is_input;
    int is_time;
    size_t offset;
    size_t count;      /* inputs only: samples in the slot */
    size_t row;        /* outputs only: which entity row inside a sample */
    size_t column;     /* outputs only: which column inside that row */
    char block[64];    /* outputs only: the result block's name */
} Variable;

typedef struct {
    fmi2Real time;
    fmi2Real step_size;
    fmi2Real grid_start;
    fmi2Real grid_step;
    size_t grid_samples;
    char resource_dir[MAX_PATH_LEN];
    char model_path[MAX_PATH_LEN];
    char case_path[MAX_PATH_LEN];
    char bindings_path[MAX_PATH_LEN];
    unsigned char* model_payload;
    size_t model_length;
    unsigned char* case_payload;
    size_t case_length;
    /*: Where the case container's blob starts, so an input binding's offset --
     *: which is relative to the blob -- addresses the right byte of the
     *: container the kernel is handed. */
    size_t case_blob_start;
    unsigned char* result;
    size_t result_length;
    size_t result_capacity;
    int solved;
    int failed;
    char error[ERROR_CAPACITY];
    Variable variables[MAX_VARIABLES];
    size_t variable_count;
    size_t input_count;
    void* kernel;
} Instance;

/* --- small helpers -------------------------------------------------------- */

static int read_file(const char* path, unsigned char** out, size_t* length) {
    FILE* file = fopen(path, "rb");
    if (file == NULL) return 0;
    if (fseek(file, 0, SEEK_END) != 0) { fclose(file); return 0; }
    long size = ftell(file);
    if (size < 0) { fclose(file); return 0; }
    if (fseek(file, 0, SEEK_SET) != 0) { fclose(file); return 0; }
    unsigned char* buffer = (unsigned char*)malloc((size_t)size);
    if (buffer == NULL) { fclose(file); return 0; }
    size_t read = fread(buffer, 1, (size_t)size, file);
    fclose(file);
    if (read != (size_t)size) { free(buffer); return 0; }
    *out = buffer;
    *length = (size_t)size;
    return 1;
}

static int join_path(char* destination, size_t capacity, const char* directory,
                     const char* name) {
    int written = snprintf(destination, capacity, "%s/%s", directory, name);
    return written > 0 && (size_t)written < capacity;
}

static void clear_error(Instance* instance) { instance->error[0] = '\0'; }

/*: Read the exporter's binding list into the instance.  Declared here because
 *: `fmi2Instantiate` uses it and its definition sits with the container
 *: readers it shares a scanner with. */
static int load_bindings(Instance* instance);

/* --- the exported symbol set --------------------------------------------- */

#ifdef _WIN32
#define FMI2_EXPORT __declspec(dllexport)
#else
#define FMI2_EXPORT __attribute__((visibility("default")))
#endif

FMI2_EXPORT const char* fmi2GetTypesPlatform(void) { return "default"; }

FMI2_EXPORT const char* fmi2GetVersion(void) { return "2.0"; }

FMI2_EXPORT fmi2Status fmi2SetDebugLogging(fmi2Component c, fmi2Boolean logging_on,
                                           size_t count, const fmi2String names[]) {
    (void)logging_on; (void)count; (void)names;
    return c == NULL ? fmi2Error : fmi2OK;
}

FMI2_EXPORT fmi2Component fmi2Instantiate(fmi2String instance_name, fmi2Type type,
                                          fmi2String guid, fmi2String resource_location,
                                          fmi2Boolean visible, fmi2Boolean logging_on) {
    (void)instance_name; (void)guid; (void)visible; (void)logging_on;
    if (type != fmi2CoSimulation) return NULL;
    if (resource_location == NULL) return NULL;

    Instance* instance = (Instance*)calloc(1, sizeof(Instance));
    if (instance == NULL) return NULL;
    strncpy(instance->resource_dir, resource_location, MAX_PATH_LEN - 1);
    instance->time = 0.0;
    instance->step_size = 0.0;

    /* All three resources travel together, so the instance solves the pair the
     * exporter was handed -- with the bindings that say where in that pair each
     * variable lives -- rather than a second reading of it. */
    if (!join_path(instance->model_path, MAX_PATH_LEN, instance->resource_dir, "model.bin") ||
        !join_path(instance->case_path, MAX_PATH_LEN, instance->resource_dir, "case.bin") ||
        !join_path(instance->bindings_path, MAX_PATH_LEN, instance->resource_dir,
                   "bindings.txt")) {
        snprintf(instance->error, ERROR_CAPACITY, "resource path too long");
        free(instance);
        return NULL;
    }
    if (!read_file(instance->model_path, &instance->model_payload, &instance->model_length)) {
        snprintf(instance->error, ERROR_CAPACITY, "cannot read %s", instance->model_path);
        free(instance);
        return NULL;
    }
    if (!read_file(instance->case_path, &instance->case_payload, &instance->case_length)) {
        snprintf(instance->error, ERROR_CAPACITY, "cannot read %s", instance->case_path);
        free(instance->model_payload);
        free(instance);
        return NULL;
    }
    if (!load_bindings(instance)) {
        free(instance->model_payload);
        free(instance->case_payload);
        free(instance);
        return NULL;
    }
    instance->result_capacity = 64u << 20;
    instance->result = (unsigned char*)malloc(instance->result_capacity);
    if (instance->result == NULL) {
        snprintf(instance->error, ERROR_CAPACITY, "cannot allocate the result buffer");
        free(instance->model_payload);
        free(instance->case_payload);
        free(instance);
        return NULL;
    }
    return (fmi2Component)instance;
}

FMI2_EXPORT void fmi2FreeInstance(fmi2Component c) {
    if (c == NULL) return;
    Instance* instance = (Instance*)c;
    if (instance->kernel != NULL) {
#ifdef _WIN32
        FreeLibrary((HMODULE)instance->kernel);
#else
        dlclose(instance->kernel);
#endif
    }
    free(instance->model_payload);
    free(instance->case_payload);
    free(instance->result);
    free(instance);
}

FMI2_EXPORT fmi2Status fmi2SetupExperiment(fmi2Component c, fmi2Boolean tolerance_defined,
                                           fmi2Real tolerance, fmi2Real start_time,
                                           fmi2Boolean stop_defined, fmi2Real stop_time) {
    (void)tolerance_defined; (void)tolerance; (void)stop_defined; (void)stop_time;
    if (c == NULL) return fmi2Error;
    ((Instance*)c)->time = start_time;
    return fmi2OK;
}

FMI2_EXPORT fmi2Status fmi2EnterInitializationMode(fmi2Component c) {
    return c == NULL ? fmi2Error : fmi2OK;
}

FMI2_EXPORT fmi2Status fmi2ExitInitializationMode(fmi2Component c) {
    return c == NULL ? fmi2Error : fmi2OK;
}

/* The kernel's containers are `MBC1`: magic(4) | version(4) | json_length(8 LE) |
 * blob_length(8 LE) | json | blob.  An input binding's offset is relative to the
 * *blob*, so the wrapper needs where that blob starts inside the container it
 * hands the kernel -- reading the header is the one way to know. */
#define CONTAINER_HEADER_SIZE 24

static int container_blob_start(const unsigned char* payload, size_t length,
                                size_t* start) {
    if (payload == NULL || length < CONTAINER_HEADER_SIZE) return 0;
    if (memcmp(payload, "MBC1", 4) != 0) return 0;
    size_t json_size = 0;
    for (int index = 7; index >= 0; --index) {
        json_size = (json_size << 8) | payload[8 + (size_t)index];
    }
    if (CONTAINER_HEADER_SIZE + json_size > length) return 0;
    *start = CONTAINER_HEADER_SIZE + json_size;
    return 1;
}

/*: Read the exporter's binding list.  Every line is `kind` followed by the
 *: numbers of one variable, so the format is parsed with `sscanf` and needs no
 *: library.  The list is authoritative: it is generated from the same bindings
 *: `modelDescription.xml` declares, so the two cannot disagree, and the
 *: `valueReference` in the file is the index the simulator passes back. */
static int load_bindings(Instance* instance) {
    FILE* file = fopen(instance->bindings_path, "rb");
    if (file == NULL) {
        snprintf(instance->error, ERROR_CAPACITY, "cannot read %s",
                 instance->bindings_path);
        return 0;
    }
    char line[256];
    while (fgets(line, sizeof(line), file) != NULL) {
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') continue;
        if (strncmp(line, "grid ", 5) == 0) {
            double start = 0.0, step = 0.0;
            long long samples = 0;
            if (sscanf(line + 5, "%lf %lf %lld", &start, &step, &samples) == 3 &&
                samples > 0) {
                instance->grid_start = (fmi2Real)start;
                instance->grid_step = (fmi2Real)step;
                instance->grid_samples = (size_t)samples;
            }
            continue;
        }
        unsigned int reference = 0;
        if (strncmp(line, "input ", 6) == 0) {
            long long offset = 0, count = 0;
            if (sscanf(line + 6, "%u %lld %lld", &reference, &offset, &count) != 3) {
                continue;
            }
            if (reference >= MAX_VARIABLES) continue;
            Variable* variable = &instance->variables[reference];
            variable->is_input = 1;
            variable->offset = (size_t)offset;
            variable->count = (size_t)count;
            instance->input_count += 1;
            continue;
        }
        if (strncmp(line, "output ", 7) == 0) {
            char block[64];
            long long row = 0, column = 0;
            if (sscanf(line + 7, "%u %63s %lld %lld", &reference, block, &row,
                       &column) != 4) {
                continue;
            }
            if (reference >= MAX_VARIABLES) continue;
            Variable* variable = &instance->variables[reference];
            strncpy(variable->block, block, sizeof(variable->block) - 1);
            variable->is_time = strcmp(block, "-") == 0;
            variable->row = (size_t)row;
            variable->column = (size_t)column;
        }
    }
    fclose(file);
    for (size_t reference = 0; reference < MAX_VARIABLES; ++reference) {
        if (instance->variables[reference].is_input ||
            instance->variables[reference].block[0] != '\0' ||
            instance->variables[reference].is_time) {
            if (reference + 1 > instance->variable_count) {
                instance->variable_count = reference + 1;
            }
        }
    }
    if (instance->variable_count == 0) {
        snprintf(instance->error, ERROR_CAPACITY,
                 "%s declares no variables", instance->bindings_path);
        return 0;
    }
    if (instance->grid_samples == 0 || instance->grid_step <= 0.0) {
        snprintf(instance->error, ERROR_CAPACITY,
                 "%s declares no usable time grid", instance->bindings_path);
        return 0;
    }
    if (!container_blob_start(instance->case_payload, instance->case_length,
                              &instance->case_blob_start)) {
        snprintf(instance->error, ERROR_CAPACITY,
                 "the case container %s is not an MBC1 container",
                 instance->case_path);
        return 0;
    }
    return 1;
}

FMI2_EXPORT fmi2Status fmi2SetReal(fmi2Component c, const fmi2ValueReference references[],
                                   size_t count, const fmi2Real values[]) {
    if (c == NULL || references == NULL || values == NULL) return fmi2Error;
    Instance* instance = (Instance*)c;
    for (size_t index = 0; index < count; ++index) {
        fmi2ValueReference reference = references[index];
        if (reference >= instance->variable_count) {
            snprintf(instance->error, ERROR_CAPACITY, "valueReference %u out of range",
                     reference);
            return fmi2Error;
        }
        Variable* variable = &instance->variables[reference];
        if (!variable->is_input) {
            snprintf(instance->error, ERROR_CAPACITY,
                     "valueReference %u is not an input of this FMU; an output is "
                     "produced by the model, not given to it",
                     reference);
            return fmi2Error;
        }
        variable->value = values[index];
        /* Write the value through to the bytes the kernel reads.  Every sample
         * of the slot takes the same number: FMI has no vector-valued real, so
         * the value a simulator sets is a constant profile over the horizon the
         * case document states.  Without this the input would be a number in
         * this struct and nothing the solver ever sees. */
        const size_t base = instance->case_blob_start + variable->offset;
        if (base + variable->count * sizeof(double) > instance->case_length) {
            snprintf(instance->error, ERROR_CAPACITY,
                     "valueReference %u addresses bytes outside the case payload",
                     reference);
            return fmi2Error;
        }
        for (size_t sample = 0; sample < variable->count; ++sample) {
            double written = (double)values[index];
            memcpy(instance->case_payload + base + sample * sizeof(double),
                   &written, sizeof(double));
        }
    }
    /* Any input the caller touched invalidates the last solve: the next
     * `fmi2DoStep` has to re-run the kernel for the answer to correspond to the
     * values now set. */
    instance->solved = 0;
    return fmi2OK;
}

/* The kernel's own result document is a `MBC1` container: the payload's header
 * is magic(4) | version(4) | json_length(8) | blob_length(8), and the result
 * *blocks* are described inside the JSON.  The wrapper does not parse JSON with
 * a library -- it looks for a block by name with a small scanner -- because the
 * only thing it needs from the result is a handful of named columns. */
#define CONTAINER_HEADER_SIZE 24

static const unsigned char* container_json(const unsigned char* payload,
                                           size_t length, size_t* json_length) {
    if (payload == NULL || length < CONTAINER_HEADER_SIZE) return NULL;
    if (memcmp(payload, "MBC1", 4) != 0) return NULL;
    size_t json_size = 0;
    for (int index = 7; index >= 0; --index) {
        json_size = (json_size << 8) | payload[8 + (size_t)index];
    }
    if (CONTAINER_HEADER_SIZE + json_size > length) return NULL;
    *json_length = json_size;
    return payload + CONTAINER_HEADER_SIZE;
}

static const unsigned char* container_blob(const unsigned char* payload, size_t length,
                                           size_t* blob_length) {
    if (payload == NULL || length < CONTAINER_HEADER_SIZE) return NULL;
    size_t json_size = 0;
    size_t blob_size = 0;
    for (int index = 7; index >= 0; --index) {
        json_size = (json_size << 8) | payload[8 + (size_t)index];
        blob_size = (blob_size << 8) | payload[16 + (size_t)index];
    }
    if (CONTAINER_HEADER_SIZE + json_size + blob_size > length) return NULL;
    *blob_length = blob_size;
    return payload + CONTAINER_HEADER_SIZE + json_size;
}

/*: Find the first occurrence of `needle` in `haystack`, bounded by `length`. */
static const unsigned char* find_bytes(const unsigned char* haystack, size_t length,
                                       const char* needle) {
    size_t needle_length = strlen(needle);
    if (needle_length == 0 || length < needle_length) return NULL;
    for (size_t index = 0; index + needle_length <= length; ++index) {
        if (memcmp(haystack + index, needle, needle_length) == 0) {
            return haystack + index;
        }
    }
    return NULL;
}

/*: Read one block's descriptor out of the result JSON.
 *
 *: This is a *scanner*, not a parser, but it is a scanner with a stated
 *: invariant: the descriptor's keys are written in alphabetical order by
 *: `block_descriptor()` (`dtype`, `length`, `name`, `offset`, `order`,
 *: `shape`), so a block's `offset` and `length` appear *before* its `name`.
 *: Scanning forward from the name therefore finds the next descriptor's
 *: fields, not this one's -- measured, and the failure is silent: the reader
 *: returns a plausible number from the wrong block.  So the search runs from
 *: the name *backwards* to the object's opening brace, and forward from there.
 *
 *: `row_extent` and `column_extent` are the descriptor's own row count and row
 *: width: a row index addresses the wrong entity without the first, and a
 *: column index the wrong quantity without the second.
 */
static int find_block(const unsigned char* json, size_t json_length, const char* name,
                      size_t* offset, size_t* length, size_t* row_extent,
                      size_t* column_extent) {
    size_t quoted_length = strlen(name) + 2;
    char* quoted = (char*)malloc(quoted_length + 1);
    if (quoted == NULL) return 0;
    snprintf(quoted, quoted_length + 1, "\"%s\"", name);
    const unsigned char* at = find_bytes(json, json_length, quoted);
    free(quoted);
    if (at == NULL) return 0;
    const unsigned char* end = json + json_length;
    /* Walk back to the '{' that opens this descriptor's object. */
    const unsigned char* open = at;
    while (open > json && *open != '{') --open;
    if (*open != '{') return 0;
    /* And forward to the matching '}'.  Nesting is counted so the closes of
     * `shape`'s array and any inner object do not end the scan early. */
    const unsigned char* close = open;
    int depth = 0;
    while (close < end) {
        if (*close == '{' || *close == '[') {
            ++depth;
        } else if (*close == '}' || *close == ']') {
            --depth;
            if (depth == 0) break;
        }
        ++close;
    }
    if (close >= end) return 0;
    const char* keys[2] = {"\"offset\"", "\"length\""};
    size_t* values[2] = {offset, length};
    for (int field = 0; field < 2; ++field) {
        /* Each field is searched from the object's own opening brace, not from
         * where the previous field ended: the keys are alphabetical, so
         * `length` precedes `offset` and a forward-only cursor would step past
         * the field it was still looking for.  The object boundary keeps the
         * search inside this descriptor either way. */
        const unsigned char* key = find_bytes(open, (size_t)(close - open), keys[field]);
        if (key == NULL) return 0;
        const unsigned char* colon = key;
        while (colon < close && *colon != ':') ++colon;
        if (colon >= close) return 0;
        ++colon;
        while (colon < close && (*colon == ' ' || *colon == '\t')) ++colon;
        size_t value = 0;
        int digits = 0;
        while (colon < close && *colon >= '0' && *colon <= '9') {
            value = value * 10 + (size_t)(*colon - '0');
            ++colon;
            ++digits;
        }
        if (digits == 0) return 0;
        *values[field] = value;
    }
    /* The shape is an array of extents in run order: a sample-major block is
     * `[samples, rows, columns]`, so the second and third extents are the row
     * count and the row width.  `length` is the descriptor's own byte size, so
     * a one-element shape falls back to the whole length as the row width. */
    *row_extent = 1;
    *column_extent = *length / sizeof(double);
    const unsigned char* shape =
        find_bytes(open, (size_t)(close - open), "\"shape\"");
    if (shape != NULL) {
        const unsigned char* array = shape;
        while (array < close && *array != '[') ++array;
        if (array < close) {
            size_t extents[4] = {0, 0, 0, 0};
            int seen = 0;
            const unsigned char* digit = array + 1;
            while (digit < close && *digit != ']' && seen < 4) {
                while (digit < close && (*digit == ' ' || *digit == ',')) ++digit;
                if (digit >= close || *digit == ']') break;
                size_t value = 0;
                int digits = 0;
                while (digit < close && *digit >= '0' && *digit <= '9') {
                    value = value * 10 + (size_t)(*digit - '0');
                    ++digit;
                    ++digits;
                }
                if (digits == 0) break;
                extents[seen] = value;
                ++seen;
            }
            if (seen == 3) {
                *row_extent = extents[1];
                *column_extent = extents[2];
            } else if (seen == 2) {
                /* A two-dimensional descriptor is `[rows, columns]`: the
                 * diagnostics ledger is written this way, one row per sample
                 * rather than one row per entity. */
                *row_extent = 1;
                *column_extent = extents[1];
            }
        }
    }
    return 1;
}

/*: Read one output variable out of the last solve's result container.
 *: The binding says which block, which entity row inside a sample, and which
 *: column inside that row; the instance's own clock says which sample.  Reading
 *: the sample from the clock rather than from the start of the block is what
 *: makes a step mean something: a run is one batch solve and the caller walks
 *: its horizon one step at a time.  The wrapper had to look up the row stride
 *: from the descriptor for the same reason -- a row index only means anything
 *: next to the extent it is indexed against. */
static int output_value(Instance* instance, const Variable* variable, fmi2Real* out) {
    if (variable->is_time) {
        *out = instance->time;
        return 1;
    }
    if (!instance->solved || instance->failed) return 0;
    size_t json_length = 0;
    const unsigned char* json =
        container_json(instance->result, instance->result_length, &json_length);
    size_t blob_length = 0;
    const unsigned char* blob =
        container_blob(instance->result, instance->result_length, &blob_length);
    if (json == NULL || blob == NULL) {
        snprintf(instance->error, ERROR_CAPACITY,
                 "the result payload for %s is not an MBC1 container", variable->block);
        return 0;
    }

    size_t offset = 0;
    size_t length = 0;
    size_t rows = 1;
    size_t columns = 1;
    if (!find_block(json, json_length, variable->block, &offset, &length, &rows,
                    &columns)) {
        snprintf(instance->error, ERROR_CAPACITY,
                 "the result carries no block %s", variable->block);
        return 0;
    }
    if (rows == 0 || columns == 0) {
        snprintf(instance->error, ERROR_CAPACITY,
                 "block %s reports an empty extent (%zu x %zu)",
                 variable->block, rows, columns);
        return 0;
    }
    if (variable->row >= rows || variable->column >= columns) {
        snprintf(instance->error, ERROR_CAPACITY,
                 "block %s is %zu x %zu and this variable asks for row %zu column %zu",
                 variable->block, rows, columns, variable->row, variable->column);
        return 0;
    }
    /* Which sample the clock is at, clamped into the run's own grid: a step
     * past the last accepted instant reports the last sample rather than
     * reading past the block. */
    size_t sample = 0;
    if (instance->grid_step > 0.0) {
        const double position =
            ((double)instance->time - (double)instance->grid_start) /
            (double)instance->grid_step;
        if (position <= 0.0) {
            sample = 0;
        } else if (position >= (double)(instance->grid_samples - 1)) {
            sample = instance->grid_samples - 1;
        } else {
            sample = (size_t)(position + 0.5);
        }
    }
    /* Sample-major addressing, which is the order the writer appends in. */
    const size_t index = (sample * rows + variable->row) * columns + variable->column;
    /* `length` is the *block's* byte size, not a position: the descriptor's own
     * length is how far the block runs, so the index is checked against it
     * rather than against `offset + index`. */
    if ((index + 1) * sizeof(double) > length) {
        snprintf(instance->error, ERROR_CAPACITY,
                 "block %s holds %zu values and this variable asks for index %zu",
                 variable->block, length / sizeof(double), index);
        return 0;
    }
    double value = 0.0;
    memcpy(&value, blob + offset + index * sizeof(double), sizeof(double));
    *out = (fmi2Real)value;
    return 1;
}

/*: Load the kernel and run the two documents.  Called from `fmi2DoStep`, so the
 *: solve happens once per step rather than once per read. */
static int solve(Instance* instance) {
    if (instance->solved) return instance->failed ? 0 : 1;
    if (instance->kernel == NULL) {
        const char* variable = getenv("SUSPENSION_MULTIBODY_KERNEL");
        char path[MAX_PATH_LEN];
        if (variable != NULL && variable[0] != '\0') {
            snprintf(path, MAX_PATH_LEN, "%s", variable);
        } else {
            /* The kernel travels beside the resources: an importing tool unpacks
             * the archive and hands us that directory, so the wrapper looks for
             * the kernel next to itself first. */
            snprintf(path, MAX_PATH_LEN, "%s", "suspension_kernel.dll");
        }
#ifdef _WIN32
        instance->kernel = (void*)LoadLibraryA(path);
#else
        instance->kernel = dlopen(path, RTLD_NOW);
#endif
        if (instance->kernel == NULL) {
            snprintf(instance->error, ERROR_CAPACITY,
                     "cannot load the native kernel from %s; set "
                     "SUSPENSION_MULTIBODY_KERNEL to its path", path);
            instance->failed = 1;
            instance->solved = 1;
            return 0;
        }
    }
#ifdef _WIN32
    kernel_run_fn run = (kernel_run_fn)GetProcAddress((HMODULE)instance->kernel,
                                                      "suspension_kernel_run");
#else
    kernel_run_fn run = (kernel_run_fn)dlsym(instance->kernel, "suspension_kernel_run");
#endif
    if (run == NULL) {
        snprintf(instance->error, ERROR_CAPACITY,
                 "the loaded library has no suspension_kernel_run entry point");
        instance->failed = 1;
        instance->solved = 1;
        return 0;
    }
    size_t capacity = instance->result_capacity;
    char error[ERROR_CAPACITY];
    error[0] = '\0';
    int status = run(instance->model_payload, instance->model_length,
                     instance->case_payload, instance->case_length,
                     instance->result, &capacity, error, ERROR_CAPACITY);
    if (status == 11 && capacity > instance->result_capacity) {
        /* The kernel asked for a bigger buffer; grow once and retry. */
        unsigned char* bigger = (unsigned char*)realloc(instance->result, capacity);
        if (bigger != NULL) {
            instance->result = bigger;
            instance->result_capacity = capacity;
            size_t retry = capacity;
            status = run(instance->model_payload, instance->model_length,
                         instance->case_payload, instance->case_length,
                         instance->result, &retry, error, ERROR_CAPACITY);
            capacity = retry;
        }
    }
    instance->result_length = capacity;
    instance->solved = 1;
    if (status != 0) {
        instance->failed = 1;
        snprintf(instance->error, ERROR_CAPACITY, "kernel status %d: %s", status,
                 error);
        return 0;
    }
    instance->failed = 0;
    return 1;
}

FMI2_EXPORT fmi2Status fmi2GetReal(fmi2Component c,
                                   const fmi2ValueReference references[], size_t count,
                                   fmi2Real values[]) {
    if (c == NULL || references == NULL || values == NULL) return fmi2Error;
    Instance* instance = (Instance*)c;
    for (size_t index = 0; index < count; ++index) {
        fmi2ValueReference reference = references[index];
        if (reference >= instance->variable_count) {
            snprintf(instance->error, ERROR_CAPACITY,
                     "valueReference %u out of range", reference);
            return fmi2Error;
        }
        const Variable* variable = &instance->variables[reference];
        if (!variable->is_input) {
            fmi2Real value = 0.0;
            if (!output_value(instance, variable, &value)) {
                /* `output_value` names the reason it could not read, and that
                 * reason is the one worth reporting: the fallbacks below only
                 * fill in when it had nothing more specific to say. */
                if (instance->error[0] == '\0') {
                    snprintf(instance->error, ERROR_CAPACITY,
                             "the result carries no usable binding for "
                             "valueReference %u", reference);
                }
                return fmi2Error;
            }
            values[index] = value;
            continue;
        }
        values[index] = variable->value;
    }
    return fmi2OK;
}

FMI2_EXPORT fmi2Status fmi2DoStep(fmi2Component c, fmi2Real current_time,
                                  fmi2Real step_size, fmi2Boolean new_step) {
    if (c == NULL) return fmi2Error;
    Instance* instance = (Instance*)c;
    if (new_step) instance->solved = 0;
    instance->step_size = step_size;
    instance->time = current_time + step_size;
    clear_error(instance);
    /* The kernel's ABI is batch: a case document carries its own time grid, so
     * one FMU step advances the co-simulation clock and the solve corresponds to
     * the run the documents describe.  Reporting a solve failure as
     * `fmi2Error` is what keeps this from looking like a model that silently
     * returns zeros. */
    if (!solve(instance)) return fmi2Error;
    return fmi2OK;
}

FMI2_EXPORT fmi2Status fmi2Terminate(fmi2Component c) {
    return c == NULL ? fmi2Error : fmi2OK;
}

FMI2_EXPORT fmi2Status fmi2Reset(fmi2Component c) {
    if (c == NULL) return fmi2Error;
    Instance* instance = (Instance*)c;
    memset(instance->variables, 0, sizeof(instance->variables));
    instance->solved = 0;
    instance->failed = 0;
    instance->time = 0.0;
    clear_error(instance);
    return fmi2OK;
}

FMI2_EXPORT fmi2Status fmi2GetStatus(fmi2Component c, fmi2StatusKind kind,
                                     fmi2Status* status) {
    (void)kind;
    if (c == NULL || status == NULL) return fmi2Error;
    *status = fmi2OK;
    return fmi2OK;
}

FMI2_EXPORT fmi2Status fmi2GetRealStatus(fmi2Component c, fmi2StatusKind kind,
                                         fmi2Real* value) {
    if (c == NULL || value == NULL) return fmi2Error;
    if (kind == fmi2LastSuccessfulTime) {
        *value = ((Instance*)c)->time;
        return fmi2OK;
    }
    return fmi2Error;
}

/* The formats this wrapper has no typed interface for.  They exist so a tool
 * probing for them finds them, and each answers with the honest "nothing
 * here". */
FMI2_EXPORT fmi2Status fmi2SetInteger(fmi2Component c,
                                      const fmi2ValueReference references[],
                                      size_t count, const fmi2Integer values[]) {
    (void)c; (void)references; (void)count; (void)values;
    return c == NULL ? fmi2Error : fmi2OK;
}

FMI2_EXPORT fmi2Status fmi2GetInteger(fmi2Component c,
                                      const fmi2ValueReference references[],
                                      size_t count, fmi2Integer values[]) {
    (void)references;
    if (c == NULL || values == NULL) return fmi2Error;
    for (size_t index = 0; index < count; ++index) values[index] = 0;
    return fmi2OK;
}

FMI2_EXPORT fmi2Status fmi2SetBoolean(fmi2Component c,
                                      const fmi2ValueReference references[],
                                      size_t count, const fmi2Boolean values[]) {
    (void)c; (void)references; (void)count; (void)values;
    return c == NULL ? fmi2Error : fmi2OK;
}

FMI2_EXPORT fmi2Status fmi2GetBoolean(fmi2Component c,
                                      const fmi2ValueReference references[],
                                      size_t count, fmi2Boolean values[]) {
    (void)references;
    if (c == NULL || values == NULL) return fmi2Error;
    for (size_t index = 0; index < count; ++index) values[index] = 0;
    return fmi2OK;
}

FMI2_EXPORT fmi2Status fmi2SetString(fmi2Component c,
                                     const fmi2ValueReference references[],
                                     size_t count, const fmi2String values[]) {
    (void)c; (void)references; (void)count; (void)values;
    return c == NULL ? fmi2Error : fmi2OK;
}

FMI2_EXPORT fmi2Status fmi2GetString(fmi2Component c,
                                     const fmi2ValueReference references[],
                                     size_t count, fmi2String values[]) {
    (void)references;
    if (c == NULL || values == NULL) return fmi2Error;
    for (size_t index = 0; index < count; ++index) values[index] = "";
    return fmi2OK;
}

/* Optional features this FMU does not provide.  Each says so rather than
 * pretending: a tool that relies on state serialisation has to be able to find
 * out, and `fmi2Error` is the interface's way of saying "not supported". */
FMI2_EXPORT fmi2Status fmi2GetFMUstate(fmi2Component c, fmi2FMUstate* state) {
    (void)c;
    if (state != NULL) *state = NULL;
    return fmi2Error;
}

FMI2_EXPORT fmi2Status fmi2SetFMUstate(fmi2Component c, fmi2FMUstate state) {
    (void)c; (void)state;
    return fmi2Error;
}

FMI2_EXPORT fmi2Status fmi2FreeFMUstate(fmi2Component c, fmi2FMUstate* state) {
    (void)c; (void)state;
    return fmi2Error;
}

FMI2_EXPORT fmi2Status fmi2SerializedFMUstateSize(fmi2Component c, fmi2FMUstate state,
                                                  size_t* size) {
    (void)c; (void)state; (void)size;
    return fmi2Error;
}

FMI2_EXPORT fmi2Status fmi2SerializeFMUstate(fmi2Component c, fmi2FMUstate state,
                                             fmi2Char* bytes, size_t size) {
    (void)c; (void)state; (void)bytes; (void)size;
    return fmi2Error;
}

FMI2_EXPORT fmi2Status fmi2DeSerializeFMUstate(fmi2Component c, const fmi2Char* bytes,
                                               size_t size, fmi2FMUstate* state) {
    (void)c; (void)bytes; (void)size; (void)state;
    return fmi2Error;
}

FMI2_EXPORT fmi2Status fmi2GetDirectionalDerivative(
    fmi2Component c, const fmi2ValueReference unknown[], size_t unknown_count,
    const fmi2ValueReference known[], size_t known_count, const fmi2Real seed[]) {
    (void)c; (void)unknown; (void)unknown_count; (void)known; (void)known_count;
    (void)seed;
    return fmi2Error;
}

/* The wrapper's own error text.  Not part of FMI; exposed so an out-of-repository
 * validator can report *why* a step failed rather than only its status code. */
FMI2_EXPORT const char* fmu_wrapper_last_error(fmi2Component c) {
    return c == NULL ? "no instance" : ((Instance*)c)->error;
}
