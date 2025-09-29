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
            return "mps"
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