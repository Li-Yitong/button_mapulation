from piper_sdk import C_PiperInterface
import time

# 初始化机械臂接口
piper = C_PiperInterface()

# 连接机械臂（默认CAN接口为can_piper，波特率1000000）
piper.ConnectPort()

if piper.get_connect_status():
    print("机械臂连接成功！")
    
    # 读取固件版本信息
    print("\n查询固件版本...")
    piper.SearchPiperFirmwareVersion()
    
    # 等待查询完成
    time.sleep(0.5)
    
    # 读取固件版本
    firmware_version = piper.GetPiperFirmwareVersion()
    print(f"机械臂固件版本: {firmware_version}")
    
    # 获取SDK版本信息
    sdk_version = piper.GetCurrentSDKVersion()
    print(f"SDK版本: {sdk_version}")
    
    # 获取接口版本信息
    interface_version = piper.GetCurrentInterfaceVersion()
    print(f"接口版本: {interface_version}")
    
    # 获取协议版本信息
    protocol_version = piper.GetCurrentProtocolVersion()
    print(f"协议版本: {protocol_version}")
else:
    print("机械臂连接失败！")