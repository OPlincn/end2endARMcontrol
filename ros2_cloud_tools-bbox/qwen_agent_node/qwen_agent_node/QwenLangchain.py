import os 
from langchain_community.chat_models.tongyi import ChatTongyi 
from langchain.agents import initialize_agent, AgentType
from langchain.memory import ConversationBufferMemory #you can also use ConversationKGMemory, ConversationSummaryMemory
from langchain.prompts import ChatPromptTemplate
# 自定义工具实现
from .MyTools import get_current_time, observe_surroundings_with_camera, detect_objects_with_camera, control_arm_action

class QwenAgent:
    def __init__(self, use_memory: bool = True) -> None:
        """
        初始化 QwenAgent，包括 LLM、工具列表、记忆和提示模板。
        """
        # 1. 初始化 LLM，model 参数根据需要可修改
        # print(rf"api_key: {os.environ['DASHSCOPE_API_KEY']}")
        self.llm = ChatTongyi(model="qwen-plus", api_key=os.environ["DASHSCOPE_API_KEY"])  

        # 2. 准备 Memory（可选）
        # memory_type 可选："buffer"（对话上下文），"kg"（知识图谱），"summary"（摘要记忆）
        self.use_memory = use_memory
        if self.use_memory:
            self.memory = ConversationBufferMemory(
                memory_key="chat_history", return_messages=True
            )
        else:
            self.memory = None

        # 3. 定义 Tools 列表
        self.tools = [
            observe_surroundings_with_camera,
            detect_objects_with_camera,
            control_arm_action,
            get_current_time
        ]

        # 4. 构建 PromptTemplate：包含系统角色、会话历史占位符和用户输入
        self.prompt = ChatPromptTemplate.from_messages([
            ("system", "你是一个资深的机器人开发工程师，精通 ROS、Python 和 C++。"),
            ("placeholder", "{chat_history}"),  # 会话历史由 Memory 填充
            ("human", "{input}"),   # 用户输入
        ])

        # 5. 使用 initialize_agent 一次性关联 LLM、Tools、Memory 和 Prompt
        self.agent_executor = initialize_agent(
            tools=self.tools,
            llm=self.llm,
            agent=AgentType.CHAT_CONVERSATIONAL_REACT_DESCRIPTION,
            memory=self.memory,
            # prompt=self.prompt,
            verbose=False,
        )

    def chat(self, user_input: str) -> str:
        """
        封装 chat 接口：输入文本，返回大模型回答的文本。
        """
        # AgentExecutor.invoke 会执行整个 agent 流程，自动处理工具调用与记忆更新
        result = self.agent_executor.invoke(input=user_input)
        # result = self.agent_executor.run(input=user_input)
        return result['output']
