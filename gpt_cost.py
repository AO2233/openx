import requests
import argparse

# Define pricing for models
# {input, cache_input, output, input_audio, output_audio}
pricing = {
    'gpt-4o': [2.5, 1.25, 10, 2.5, 10],
    'gpt-4o-2024-08-06': [2.5, 1.25, 10, 2.5, 10],
    
    'gpt-4o-mini': [0.150, 0.0075, 0.6, 0.150, 0.6],
    'gpt-4o-mini-2024-07-18': [0.150, 0.0075, 0.6, 0.150, 0.6],  
    
    'gpt-4o-realtime-preview': [5, 2.5, 20, 40, 80],
    'gpt-4o-realtime-preview-2024-12-17': [5, 2.5, 20, 40, 80],
}

def fetch_usage(date: str, api_key: str):
    url = f"https://api.openai.com/v1/usage?date={date}"
    headers = {'Authorization': f"Bearer {api_key}"}
    response = requests.get(url, headers=headers)
    return response.json()['data']

def get_token(entry):
    gen_t = entry["n_generated_tokens_total"] # output text
    ctx_t = entry["n_context_tokens_total"] # input text
    cache_t = entry["n_cached_context_tokens_total"]    # cached input text
    audio_ctx_t = entry["n_context_audio_tokens_total"]   # input audio
    audio_gen_t = entry["n_generated_audio_tokens_total"]   # output audio
    ctx_t = ctx_t - cache_t # uncached input text
   
    return [ctx_t, cache_t, gen_t, audio_ctx_t, audio_gen_t]

def calculate_tokens(data):
    token_sum = {
        'ctx_t': 0,
        'cache_t': 0,
        'gen_t': 0,
        'audio_ctx_t': 0,
        'audio_gen_t': 0
    }

    for entry in data:
        ctx_t, cache_t, gen_t, audio_ctx_t, audio_gen_t = get_token(entry)
        token_sum['ctx_t'] += ctx_t
        token_sum['cache_t'] += cache_t
        token_sum['gen_t'] += gen_t
        token_sum['audio_ctx_t'] += audio_ctx_t
        token_sum['audio_gen_t'] += audio_gen_t

    return token_sum['ctx_t'], token_sum['cache_t'], token_sum['gen_t'], token_sum['audio_ctx_t'], token_sum['audio_gen_t']

def calculate_cost(data, pricing):
    total_cost = 0
    for entry in data:
        if entry['snapshot_id'] in pricing.keys():
            token_counts = get_token(entry)
            total_cost += sum(t * r for t, r in zip(token_counts, pricing[entry['snapshot_id']]))
        else:
            print(f"Model {entry['snapshot_id']} not found in pricing dict, skipping. Please check https://openai.com/api/pricing/ and add the model to the pricing dict.")
    return total_cost / 1e6

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="program to get cost from OpenAI")
    parser.add_argument("--date", "-d", type=str, required=True, help="date you want to search, like 2025-01-01")
    parser.add_argument("--key", "-k", type=str, required=True, help="tell me your key, like sk-1234")
    args = parser.parse_args()

    content = fetch_usage(args.date, args.key)
    print("==========\n" + \
        f"uncached input: {calculate_tokens(content)[0]}\n" + \
        f"cached input: {calculate_tokens(content)[1]}\n" + \
        f"output: {calculate_tokens(content)[2]}\n"+ \
        f"input audio: {calculate_tokens(content)[3]}\n"+ \
        f"output audio: {calculate_tokens(content)[4]}\n" + \
        "==========")
    print(f"Your Cost: $ {calculate_cost(content, pricing)}")
    