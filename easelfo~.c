/**
 * easelfo~ - Signal-rate LFO with easing functions for Max/MSP
 * 
 * This external generates an LFO using various easing functions commonly used
 * in animation and motion design, providing more musical modulation curves.
 * 
 * Inlets:
 *   1. Frequency in Hz (signal or float) - this should also accept a bang to reset the phase
 *   2. Easing function selection (int, 0-11)
 * 
 * Outlets:
 *   1. LFO output (signal, -1.0 to 1.0)
 */

#include "ext.h"
#include "ext_obex.h"
#include "z_dsp.h"
#include <math.h>

#define PI 3.14159265358979323846

typedef struct _easelfo {
    t_pxobject ob;              // MSP object header
    double phase;               // Current phase (0.0 to 1.0)
    double sr;                  // Sample rate
    double sr_inv;              // 1.0 / sample rate
    long easing_func;           // Selected easing function
    void *proxy1;               // Proxy for second inlet (easing function)
    void *proxy2;               // Proxy for third inlet (phase offset)
    long proxy_inletnum;        // Inlet number for proxy
    double freq_float;          // Default frequency when no signal connected
    double phase_offset;        // Phase offset (0.0 to 1.0)
    long mirror_mode;           // Mirror mode (0 = off, 1 = on)
} t_easelfo;

// Function prototypes
void *easelfo_new(t_symbol *s, long argc, t_atom *argv);
void easelfo_free(t_easelfo *x);
void easelfo_dsp64(t_easelfo *x, t_object *dsp64, short *count, double samplerate, 
                   long maxvectorsize, long flags);
void easelfo_perform64(t_easelfo *x, t_object *dsp64, double **ins, long numins, 
                       double **outs, long numouts, long sampleframes, long flags, void *userparam);
void easelfo_int(t_easelfo *x, long n);
void easelfo_float(t_easelfo *x, double f);
void easelfo_bang(t_easelfo *x);
void easelfo_mirror(t_easelfo *x, long n);
void easelfo_assist(t_easelfo *x, void *b, long m, long a, char *s);

// Easing function prototypes
double ease_linear(double t);
double ease_sine_in(double t);
double ease_sine_out(double t);
double ease_sine_inout(double t);
double ease_quad_in(double t);
double ease_quad_out(double t);
double ease_quad_inout(double t);
double ease_cubic_in(double t);
double ease_cubic_out(double t);
double ease_cubic_inout(double t);
double ease_expo_in(double t);
double ease_expo_out(double t);

// Class pointer
static t_class *easelfo_class = NULL;

// Main function
void ext_main(void *r) {
    t_class *c;
    
    c = class_new("easelfo~", (method)easelfo_new, (method)easelfo_free,
                  sizeof(t_easelfo), 0L, A_GIMME, 0);
    
    class_addmethod(c, (method)easelfo_dsp64, "dsp64", A_CANT, 0);
    class_addmethod(c, (method)easelfo_assist, "assist", A_CANT, 0);
    class_addmethod(c, (method)easelfo_int, "int", A_LONG, 0);
    class_addmethod(c, (method)easelfo_float, "float", A_FLOAT, 0);
    class_addmethod(c, (method)easelfo_bang, "bang", A_CANT, 0);
    class_addmethod(c, (method)easelfo_mirror, "mirror", A_LONG, 0);
    
    class_dspinit(c);
    class_register(CLASS_BOX, c);
    easelfo_class = c;
}

// Constructor
void *easelfo_new(t_symbol *s, long argc, t_atom *argv) {
    t_easelfo *x = (t_easelfo *)object_alloc(easelfo_class);
    
    if (x) {
        dsp_setup((t_pxobject *)x, 1);  // 1 signal inlet (frequency)
        outlet_new(x, "signal");         // 1 signal outlet
        
        // Initialize
        x->phase = 0.0;
        x->sr = sys_getsr();
        x->sr_inv = 1.0 / x->sr;
        x->easing_func = 0;  // Default to linear
        x->freq_float = 1.0;  // Default 1 Hz frequency
        x->phase_offset = 0.0;  // Default no phase offset
        x->mirror_mode = 0;     // Default mirror mode off
        
        // Create proxy inlets for proper 3-inlet design
        x->proxy2 = proxy_new((t_object *)x, 2, &x->proxy_inletnum);  // Phase offset (rightmost)
        x->proxy1 = proxy_new((t_object *)x, 1, &x->proxy_inletnum);  // Easing function (middle)
        
        // Process arguments
        if (argc >= 1 && atom_gettype(argv) == A_LONG) {
            x->easing_func = atom_getlong(argv);
            if (x->easing_func < 0) x->easing_func = 0;
            if (x->easing_func > 11) x->easing_func = 11;
        }
    }
    
    return x;
}

// Destructor
void easelfo_free(t_easelfo *x) {
    dsp_free((t_pxobject *)x);
    object_free(x->proxy1);
    object_free(x->proxy2);
}

// DSP setup
void easelfo_dsp64(t_easelfo *x, t_object *dsp64, short *count, double samplerate, 
                   long maxvectorsize, long flags) {
    x->sr = samplerate;
    x->sr_inv = 1.0 / samplerate;
    object_method(dsp64, gensym("dsp_add64"), x, easelfo_perform64, 0, NULL);
}

// Signal processing
void easelfo_perform64(t_easelfo *x, t_object *dsp64, double **ins, long numins, 
                       double **outs, long numouts, long sampleframes, long flags, void *userparam) {
    double *freq_in = ins[0];   // Frequency input
    double *out = outs[0];       // Output
    long n = sampleframes;
    double phase = x->phase;
    double sr_inv = x->sr_inv;
    long easing = x->easing_func;
    
    // Function pointer array for easing functions
    double (*easing_functions[12])(double) = {
        ease_linear, ease_sine_in, ease_sine_out, ease_sine_inout,
        ease_quad_in, ease_quad_out, ease_quad_inout,
        ease_cubic_in, ease_cubic_out, ease_cubic_inout,
        ease_expo_in, ease_expo_out
    };
    
    while (n--) {
        // Use signal input or default frequency
        double freq = *freq_in++;
        if (freq == 0.0) {
            freq = x->freq_float;
        }
        
        // Halve frequency in mirror mode to maintain consistent timing
        if (x->mirror_mode) {
            freq *= 0.5;
        }
        
        // Update phase
        phase += freq * sr_inv;
        
        // Wrap phase
        while (phase >= 1.0) phase -= 1.0;
        while (phase < 0.0) phase += 1.0;
        
        // Apply phase offset
        double offset_phase = phase + x->phase_offset;
        while (offset_phase >= 1.0) offset_phase -= 1.0;
        while (offset_phase < 0.0) offset_phase += 1.0;
        
        // Apply mirror mode if enabled
        double final_phase = offset_phase;
        if (x->mirror_mode) {
            // Mirror the phase: 0->1->0 instead of 0->1
            if (final_phase <= 0.5) {
                final_phase = final_phase * 2.0;  // 0.0-0.5 becomes 0.0-1.0
            } else {
                final_phase = (1.0 - final_phase) * 2.0;  // 0.5-1.0 becomes 1.0-0.0
            }
        }
        
        // Apply easing function
        double eased = easing_functions[easing](final_phase);
        
        // Map to bipolar output (-1 to 1)
        *out++ = (eased * 2.0) - 1.0;
    }
    
    x->phase = phase;
}

// Handle integer input
void easelfo_int(t_easelfo *x, long n) {
    long inlet = proxy_getinlet((t_object *)x);
    
    if (inlet == 1) {  // Second inlet - easing function selection
        x->easing_func = n;
        if (x->easing_func < 0) x->easing_func = 0;
        if (x->easing_func > 11) x->easing_func = 11;
    } else if (inlet == 2) {  // Third inlet - phase offset (treat int as float)
        x->phase_offset = (double)n;
        // Wrap phase offset to 0.0-1.0 range
        while (x->phase_offset >= 1.0) x->phase_offset -= 1.0;
        while (x->phase_offset < 0.0) x->phase_offset += 1.0;
    }
}

// Handle float input
void easelfo_float(t_easelfo *x, double f) {
    long inlet = proxy_getinlet((t_object *)x);
    
    if (inlet == 0) {  // First inlet - frequency
        x->freq_float = f;
    } else if (inlet == 1) {  // Second inlet - easing function
        easelfo_int(x, (long)f);
    } else if (inlet == 2) {  // Third inlet - phase offset
        x->phase_offset = f;
        // Wrap phase offset to 0.0-1.0 range
        while (x->phase_offset >= 1.0) x->phase_offset -= 1.0;
        while (x->phase_offset < 0.0) x->phase_offset += 1.0;
    }
}

// Handle bang input - reset phase
void easelfo_bang(t_easelfo *x) {
    long inlet = proxy_getinlet((t_object *)x);
    
    if (inlet == 0) {  // First inlet - reset phase
        x->phase = 0.0;
    }
}

// Handle mirror message
void easelfo_mirror(t_easelfo *x, long n) {
    x->mirror_mode = (n != 0) ? 1 : 0;  // Any non-zero = mirror on
}

// Assist method
void easelfo_assist(t_easelfo *x, void *b, long m, long a, char *s) {
    if (m == ASSIST_INLET) {
        switch (a) {
            case 0:
                sprintf(s, "(signal/float/bang) Frequency in Hz, bang to reset phase");
                break;
            case 1:
                sprintf(s, "(int) Easing function (0-11)");
                break;
            case 2:
                sprintf(s, "(float) Phase offset (0.0-1.0)");
                break;
        }
    } else {  // ASSIST_OUTLET
        sprintf(s, "(signal) LFO output (-1 to 1)");
    }
}

// Easing functions implementation

double ease_linear(double t) {
    return t;
}

double ease_sine_in(double t) {
    return 1.0 - cos((t * PI) / 2.0);
}

double ease_sine_out(double t) {
    return sin((t * PI) / 2.0);
}

double ease_sine_inout(double t) {
    return -(cos(PI * t) - 1.0) / 2.0;
}

double ease_quad_in(double t) {
    return t * t;
}

double ease_quad_out(double t) {
    return t * (2.0 - t);
}

double ease_quad_inout(double t) {
    return t < 0.5 ? 2.0 * t * t : 1.0 - pow(-2.0 * t + 2.0, 2.0) / 2.0;
}

double ease_cubic_in(double t) {
    return t * t * t;
}

double ease_cubic_out(double t) {
    return 1.0 - pow(1.0 - t, 3.0);
}

double ease_cubic_inout(double t) {
    return t < 0.5 ? 4.0 * t * t * t : 1.0 - pow(-2.0 * t + 2.0, 3.0) / 2.0;
}

double ease_expo_in(double t) {
    return t == 0.0 ? 0.0 : pow(2.0, 10.0 * t - 10.0);
}

double ease_expo_out(double t) {
    return t == 1.0 ? 1.0 : 1.0 - pow(2.0, -10.0 * t);
}