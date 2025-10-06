import torch
from transformers import AutoTokenizer, AutoModelForCausalLM, pipeline
from abc import ABC, abstractmethod

class LLM(ABC):

    """Parent class for all LLM models"""
    
    def __init__(self, config):
        self.config = config
        self.model_name = config["CHAT_MODEL"]  # To be set by child class
        self.model = None
        self.tokenizer = None
        self.pipeline = None
        self.device = self._get_device()
        self.init_function()  # Call initialization
    
    def _get_device(self):
        """Determine best available device"""
        if torch.cuda.is_available():
            return torch.cuda.current_device()
        elif torch.backends.mps.is_available():
            return "mps"    # technically, used for mac gpu but here only for completion
        return "cpu"
    
    @abstractmethod
    def init_function(self):
        """Initialize model (to be implemented by child classes)"""
        raise NotImplementedError()
    
    @abstractmethod
    def chat(self, prompt):
        """Generate response (to be implemented by child classes)"""
        raise NotImplementedError()


class GPT2(LLM):
    """GPT-2 model implementation"""
    
    def __init__(self, config):
        super().__init__(config)  # This will call init_function()
        self.model_name = "sshleifer/tiny-gpt2"  # Set model name before parent init

    
    def init_function(self):
        """Initialize GPT-2 model"""
        print(f"Loading {self.model_name}...")
        
        # Load tokenizer
        self.tokenizer = AutoTokenizer.from_pretrained(
            self.model_name,
            cache_dir=self.config.get("CACHE_DIR")
        )
        self.tokenizer.pad_token = self.tokenizer.eos_token
        
        # Load model
        self.model = AutoModelForCausalLM.from_pretrained(
            self.model_name,
            cache_dir=self.config.get("CACHE_DIR"),
            torch_dtype="auto"
        )
        
        # Create pipeline
        self.pipeline = pipeline(
            "text-generation",
            model=self.model,
            tokenizer=self.tokenizer,
            device=self.device,
            return_full_text=True
        )
        
        print(f"{self.model_name} loaded!")
    
    def chat(self, prompt):
        """Generate text completion"""
        generation_params = {
            #"max_new_tokens": 100,
            "temperature": 0.7,
            "top_p": 0.9,
            "do_sample": True
        }
        
        response = self.pipeline(prompt, **generation_params)
        
        # Extract only generated text
        generated = response[0]["generated_text"]
        if prompt in generated:
            generated = generated[len(prompt):].strip()
        
        return generated
    



class Qwen3(LLM):
    """Qwen3-4B-Instruct-2507 model implementation"""

    def __init__(self, config):
        super().__init__(config)
        self.model_name = "Qwen/Qwen3-4B-Instruct-2507"  # Adjust based on actual model availability


    def init_function(self):
        """Initialize Qwen3 model"""
        print(f"Loading {self.model_name}...")

        self.tokenizer = AutoTokenizer.from_pretrained(
            self.model_name,
            trust_remote_code=True,
            cache_dir=self.config.get("CACHE_DIR")
        )

        self.model = AutoModelForCausalLM.from_pretrained(
            self.model_name,
            torch_dtype="auto",
            trust_remote_code=True,
             #device_map="auto",  # Optional: offloads to available devices
            cache_dir=self.config.get("CACHE_DIR")
        )

        self.pipeline = pipeline(
            "text-generation",
            model=self.model,
            tokenizer=self.tokenizer,
            device=self.device,
            return_full_text=False  # Let us handle formatting output
        )

        print(f"{self.model_name} loaded!")

    def chat(self, prompt):
        """Generate a response using tokenizer's chat_template"""

        # Single-round message history # TODO: system+user??
        messages = [
            {"role": "user", "content": prompt}
        ]

        try:
            formatted_prompt = self.tokenizer.apply_chat_template(
                messages,
                tokenize=False,
                add_generation_prompt=True
            )
        except Exception as e:
            raise RuntimeError(f"Failed to apply chat template: {e}")

        generation_params = {
            "max_new_tokens": 512,
            "temperature": 0.7,
            "top_p": 0.9,
            "do_sample": True,
            "eos_token_id": self.tokenizer.eos_token_id
        }

        response = self.pipeline(formatted_prompt, **generation_params)
        generated = response[0]["generated_text"]

        # Strip input prompt from generated text if it exists
        if formatted_prompt in generated:
            generated = generated[len(formatted_prompt):].strip()

        return generated



class Mistral(LLM):
    """Mistral-7B-OpenHermes-2.5 model implementation"""

    def __init__(self, config):
        super().__init__(config)
        self.model_name = "teknium/OpenHermes-2.5-Mistral-7B"

    def init_function(self):
        """Initialize Mistral model"""
        print(f"Loading {self.model_name}...")

        self.tokenizer = AutoTokenizer.from_pretrained(
            self.model_name,
            trust_remote_code=True,
            cache_dir=self.config.get("CACHE_DIR")
        )

        # Set pad_token if not already set (common for Mistral models)
        if self.tokenizer.pad_token is None:
            self.tokenizer.pad_token = self.tokenizer.eos_token

        self.model = AutoModelForCausalLM.from_pretrained(
            self.model_name,
            torch_dtype=torch.float16,  # Use float16 for efficiency
            trust_remote_code=True,
            cache_dir=self.config.get("CACHE_DIR")
        )

        # Move model to device
        self.model.to(self.device)

        self.pipeline = pipeline(
            "text-generation",
            model=self.model,
            tokenizer=self.tokenizer,
            device=self.device,
            return_full_text=False
        )

        print(f"{self.model_name} loaded!")

    def chat(self, prompt):
        """Generate a response using Mistral's chat format"""

        # OpenHermes uses ChatML format
        messages = [
            {"role": "user", "content": prompt}
        ]

        try:
            # Apply chat template (OpenHermes supports ChatML)
            formatted_prompt = self.tokenizer.apply_chat_template(
                messages,
                tokenize=False,
                add_generation_prompt=True
            )
        except Exception as e:
            # Fallback to manual ChatML formatting if template not available
            print(f"Chat template not available, using manual formatting: {e}")
            formatted_prompt = f"<|im_start|>user\n{prompt}<|im_end|>\n<|im_start|>assistant\n"

        generation_params = {
            "max_new_tokens": 512,
            "temperature": 0.7,
            "top_p": 0.9,
            "do_sample": True,
            "eos_token_id": self.tokenizer.eos_token_id,
            "pad_token_id": self.tokenizer.pad_token_id
        }

        response = self.pipeline(formatted_prompt, **generation_params)
        generated = response[0]["generated_text"]

        # Clean up the response
        generated = generated.strip()
        
        # Remove any potential ChatML end tokens
        if "<|im_end|>" in generated:
            generated = generated.split("<|im_end|>")[0].strip()

        return generated