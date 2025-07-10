# ros2_cloud_tools

这个包整合了阿里云百炼平台提供的ASR / TTS / LLM with Langchain / Translate 功能
还有 DOSOD object2text 的节点.
方便在RDK Series开发板上调用阿里云百炼平台的API!
使用之前一定要设置DashScope API Key哦!
### DashScope API Key (CRITICAL)

Before running any nodes in this package, you **MUST** set your DashScope API key as an environment variable. The node reads this variable for authentication.

```bash
export DASHSCOPE_API_KEY='sk-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx' # Replace with your actual key
```
### 
