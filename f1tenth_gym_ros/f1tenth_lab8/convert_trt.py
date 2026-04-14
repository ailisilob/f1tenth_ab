import tensorrt as trt
import numpy as np
import time

TRT_LOGGER = trt.Logger(trt.Logger.WARNING)

def build_engine(onnx_path, engine_path, fp16=False):
    with trt.Builder(TRT_LOGGER) as builder:
        explicit_batch = 1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
        network = builder.create_network(explicit_batch)

        parser = trt.OnnxParser(network, TRT_LOGGER)
        config = builder.create_builder_config()
        config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1 << 28)

        if fp16:
            config.set_flag(trt.BuilderFlag.FP16)
            print("Building FP16 engine...")
        else:
            print("Building FP32 engine...")

        with open(onnx_path, 'rb') as f:
            if not parser.parse(f.read()):
                for i in range(parser.num_errors):
                    print(f"ONNX parse error: {parser.get_error(i)}")
                return False

        # Add optimization profile for dynamic batch size
        profile = builder.create_optimization_profile()
        # Input shape: (batch, 3, 180, 320)
        profile.set_shape('input',
                          min=(1, 3, 180, 320),
                          opt=(1, 3, 180, 320),
                          max=(1, 3, 180, 320))
        config.add_optimization_profile(profile)

        print(f"Network inputs: {network.num_inputs}")
        print(f"Network outputs: {network.num_outputs}")

        serialized = builder.build_serialized_network(network, config)
        if serialized is None:
            print("Failed to build engine")
            return False

        with open(engine_path, 'wb') as f:
            f.write(serialized)
        print(f"Engine saved to {engine_path}")
        return True

if __name__ == '__main__':
    onnx_path = 'model_final.onnx'

    # Build FP32 engine and measure time
    t0 = time.time()
    build_engine(onnx_path, 'model_fp32.trt', fp16=False)
    fp32_time = time.time() - t0
    print(f"FP32 build time: {fp32_time:.1f}s")

    # Build FP16 engine and measure time
    t0 = time.time()
    build_engine(onnx_path, 'model_fp16.trt', fp16=True)
    fp16_time = time.time() - t0
    print(f"FP16 build time: {fp16_time:.1f}s")

    print("Done! Both engines saved.")