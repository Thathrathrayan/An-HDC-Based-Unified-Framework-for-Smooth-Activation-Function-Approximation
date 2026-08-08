import numpy as np
import math
import matplotlib.pyplot as plt

# =====================================================================
# CONFIGURATION SETTINGS: D=1760, BLOCK_SIZE=22 (80 Blocks Total)
# =====================================================================
SELECTED_D = 1760          # Hypervector Dimension
SELECTED_BLOCK_SIZE = 22   # Block Size (22 bits * 80 blocks = 1760 bits)

# Safety check to guarantee exact tiling of 80 blocks
#assert SELECTED_D // SELECTED_BLOCK_SIZE == 80, "Error: SELECTED_D / SELECTED_BLOCK_SIZE must equal exactly 80!"

# =====================================================================
# 1. OPTIMIZED DENSE GREEDY CODEBOOK GENERATOR
# =====================================================================
def generate_max_distance_vectors(num_vectors=80, num_bits=22, candidates_per_step=2000):
    """
    Generates a set of binary vectors maximizing the minimum Hamming distance
    using a strict dense (50% density) probability model and greedy candidate selection.
    """
    rng = np.random.default_rng(42)
    first_vector = rng.integers(0, 2, size=num_bits, dtype=np.int8)
    selected_vectors = [first_vector]
    
    for _ in range(1, num_vectors):
        candidates = rng.integers(0, 2, size=(candidates_per_step, num_bits), dtype=np.int8)
        current_matrix = np.array(selected_vectors)
        
        mismatches = candidates[:, None, :] != current_matrix[None, :, :]
        pairwise_distances = np.sum(mismatches, axis=-1) 
        
        min_distances_per_candidate = np.min(pairwise_distances, axis=1)
        best_idx = np.argmax(min_distances_per_candidate)
        selected_vectors.append(candidates[best_idx])
        
    final_set = np.array(selected_vectors)
    
    # Calculate minimum distance across codebook
    final_diffs = final_set[:, None, :] != final_set[None, :, :]
    final_distances = np.sum(final_diffs, axis=-1)
    np.fill_diagonal(final_distances, 999) # Ignore self-distance
    
    actual_min_distance = np.min(final_distances)
    return final_set, actual_min_distance

# =====================================================================
# 2. HDC PIPELINE EMULATION
# =====================================================================
def get_shifted_vector(start_idx, luts, D, block_size):
    """Constructs a D-bit vector by tiling 80 block_size-bit unique LUTs sequentially."""
    num_blocks = D // block_size
    blocks = []
    for k in range(num_blocks):  
        lut_idx = (start_idx + k) % len(luts)
        blocks.append(luts[lut_idx])
    return np.concatenate(blocks)

def bind(*vectors):
    """Bitwise XOR binding operator."""
    result = vectors[0]
    for v in vectors[1:]:
        result = np.bitwise_xor(result, v)
    return result

def get_quantized_answer(x, func):
    """Target activation mathematical calculation snapped to output vocab levels."""
    if func == 'Swish':
        ans = x / (1 + np.exp(-x))
    elif func == 'GELU':
        ans = 0.5 * x * (1 + math.erf(x / 1.41421))
    elif func == 'Tanh':
        ans = x * np.tanh(np.exp(x))
    else:
        return 0.0
    ans = max(-0.35, min(0.0, ans))
    return round(ans * 100) / 100.0

def run_hdc_simulation(D, block_size, x_sweep, unique_luts_parsed):
    """
    Executes training and inference using Holographic Perceptron Learning.
    """
    inputs = [round(x, 1) for x in np.arange(-4.0, 0.1, 0.1)]
    functions = ['Swish', 'GELU', 'Tanh']
    vocab_vals = [round(x, 2) for x in np.arange(-0.35, 0.01, 0.01)]
    
    input_vectors = {x: get_shifted_vector(idx, unique_luts_parsed, D, block_size) for idx, x in enumerate(inputs)}
    func_keys = {
        'Swish': get_shifted_vector(41, unique_luts_parsed, D, block_size),
        'GELU': get_shifted_vector(42, unique_luts_parsed, D, block_size),
        'Tanh': get_shifted_vector(43, unique_luts_parsed, D, block_size)
    }
    output_dictionary = {val: get_shifted_vector(idx + 44, unique_luts_parsed, D, block_size) for idx, val in enumerate(vocab_vals)}
    
    samples = []
    for x in inputs:
        for f in functions:
            q_ans = get_quantized_answer(x, f)
            samples.append((x, f, q_ans))
            
    def to_bipolar(binary_vec):
        return 2 * binary_vec - 1
        
    def to_binary(bipolar_vec):
        return (bipolar_vec >= 0).astype(np.int8)

    print(f"   -> Training Super-Vector using Dense Perceptron Learning (D={D})...")
    super_vector_accum = np.zeros(D, dtype=np.int32)
    
    epochs = 5000
    learning_rate = 1
    
    for epoch in range(epochs):
        errors = 0
        for x, f, q_ans in samples:
            query = bind(input_vectors[x], func_keys[f])
            
            current_super_binary = to_binary(super_vector_accum)
            noisy_answer = bind(current_super_binary, query)
            
            best_match_val = None
            best_match_score = -1
            
            for dict_val, clean_ans_vector in output_dictionary.items():
                hamming_distance = np.sum(noisy_answer != clean_ans_vector)
                if best_match_score == -1 or hamming_distance < best_match_score:
                    best_match_score = hamming_distance
                    best_match_val = dict_val
            
            if best_match_val != q_ans:
                errors += 1
                target_bound = bind(query, output_dictionary[q_ans])
                wrong_bound = bind(query, output_dictionary[best_match_val])
                
                target_bipolar = to_bipolar(target_bound)
                wrong_bipolar = to_bipolar(wrong_bound)
                
                super_vector_accum += learning_rate * (target_bipolar - wrong_bipolar)
                
        if errors == 0:
            print(f"      * Converged early at epoch {epoch+1} with 0 errors!")
            break
    else:
        print(f"      * Completed {epochs} epochs (Remaining training errors: {errors})")
        
    super_vector = to_binary(super_vector_accum)
    
    def execute_hdc_query(quantized_input, user_func):
        lookup_val = round(float(quantized_input), 1)
        if lookup_val == 0.0: lookup_val = 0.0
        
        query = bind(input_vectors[lookup_val], func_keys[user_func])
        noisy_answer = bind(super_vector, query)
        
        best_match_val = None
        best_match_score = -1
        
        for dict_val, clean_ans_vector in output_dictionary.items():
            hamming_distance = np.sum(noisy_answer != clean_ans_vector)
            if best_match_score == -1 or hamming_distance < best_match_score:
                best_match_score = hamming_distance
                best_match_val = dict_val
                
        return best_match_val

    def get_hdc_prediction(user_input, user_func):
        if user_func == 'Tanh':
            if user_input >= 0:
                return user_input
            elif user_input < -4.0:
                return 0.0
            else:
                q_x = round(user_input * 10) / 10.0
                return execute_hdc_query(q_x, user_func)
        else:
            if user_input < -4.0:
                return 0.0
            elif user_input > 4.0:
                return user_input
            else:
                q_x_neg = round(-abs(user_input) * 10) / 10.0
                hdc_val = execute_hdc_query(q_x_neg, user_func)
                if user_input >= 0:
                    return user_input + hdc_val
                else:
                    return hdc_val

    swish_preds = [get_hdc_prediction(x, 'Swish') for x in x_sweep]
    gelu_preds  = [get_hdc_prediction(x, 'GELU') for x in x_sweep]
    tanh_preds  = [get_hdc_prediction(x, 'Tanh') for x in x_sweep]
    
    return swish_preds, gelu_preds, tanh_preds, super_vector

# =====================================================================
# 3. RUN SIMULATION FOR SELECTED SCALE
# =====================================================================
x_sweep = np.linspace(-5.0, 5.0, 150)

true_swish = [x / (1 + np.exp(-x)) for x in x_sweep]
true_gelu  = [0.5 * x * (1 + math.erf(x / 1.41421)) for x in x_sweep]
true_tanh  = [x * np.tanh(np.exp(x)) for x in x_sweep]

print("="*60)
print(f"RUNNING PARAMETERS FOR SELECTED CONFIG: D={SELECTED_D}, BLOCK={SELECTED_BLOCK_SIZE}")
print("="*60)

selected_luts, actual_min_dist = generate_max_distance_vectors(
    num_vectors=80, 
    num_bits=SELECTED_BLOCK_SIZE
)

unique_rows = np.unique(selected_luts, axis=0)
is_completely_unique = len(unique_rows) == len(selected_luts)

print(f"Uniqueness Asserted: {'PASSED (80/80 Unique)' if is_completely_unique else 'FAILED'}")
print(f"Min Pairwise Hamming Distance: {actual_min_dist} bits ({actual_min_dist/SELECTED_BLOCK_SIZE*100:.1f}%)")

swish_preds, gelu_preds, tanh_preds, super_vec = run_hdc_simulation(
    SELECTED_D, SELECTED_BLOCK_SIZE, x_sweep, selected_luts
)

print("-" * 60)

# =====================================================================
# 4. RENDER GRAPH
# =====================================================================
fig, axs = plt.subplots(1, 3, figsize=(15, 4.5))
fig.suptitle(f"Selected Config Analysis: D={SELECTED_D} | Block Size={SELECTED_BLOCK_SIZE}-bit (Dense 50%)", fontsize=14, fontweight='bold')

axs[0].plot(x_sweep, true_swish, label='True Swish', color='blue', linewidth=2)
axs[0].plot(x_sweep, swish_preds, label='HDC Hardware Swish', color='red', linestyle='--', linewidth=2)
axs[0].set_title('Swish Curve')
axs[0].grid(True, linestyle=':', alpha=0.6)
axs[0].legend(fontsize=9)

axs[1].plot(x_sweep, true_gelu, label='True GELU', color='green', linewidth=2)
axs[1].plot(x_sweep, gelu_preds, label='HDC Hardware GELU', color='red', linestyle='--', linewidth=2)
axs[1].set_title('GELU Curve')
axs[1].grid(True, linestyle=':', alpha=0.6)
axs[1].legend(fontsize=9)

axs[2].plot(x_sweep, true_tanh, label='True Tanhexp', color='purple', linewidth=2)
axs[2].plot(x_sweep, tanh_preds, label='HDC Hardware Tanhexp', color='red', linestyle='--', linewidth=2)
axs[2].set_title('Tanhexp Curve')
axs[2].grid(True, linestyle=':', alpha=0.6)
axs[2].legend(fontsize=9)

for ax in axs:
    ax.set_xlabel('Input (x)')
    ax.set_ylabel('Output (y)')
    ax.set_ylim([-1.0, 5.0])
    
plt.tight_layout()

# =====================================================================
# 5. DYNAMIC VERILOG LOOKUP STRINGS EXPORTER
# =====================================================================
print("\n" + "="*80)
print(f"VERILOG INITIALIZATION STRINGS (D={SELECTED_D}, Block={SELECTED_BLOCK_SIZE} Configuration)")
print("="*80)
for idx, lut in enumerate(selected_luts):
    print(f"{SELECTED_BLOCK_SIZE}'b{''.join(map(str, lut))},")

print(f"\nsuper_vector_mem = {SELECTED_D}'b{''.join(map(str, super_vec))};")
print("="*80 + "\n")

plt.show()
