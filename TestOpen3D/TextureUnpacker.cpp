#include "TextureUnpacker.h"
#include "ErrorLogger.h"

bool TextureUnpacker::UnpackTexture(geometry::Image& im, geometry::TriangleMesh& mesh, bool debug)
{
    UvpOperationInputT uvpInput;

    uvpInput.m_pDeviceId = "cpu";
    uvpInput.m_RenderResult = true;
    uvpInput.m_RenderInvalidIslands = true;
    uvpInput.m_RealtimeSolution = true;
    uvpInput.m_Benchmark = true;
    uvpInput.m_Opcode = UVP_OPCODE::PACK;

	UvpOpExecutorT opExecutor(debug);

    UvpOperationInputT reportVersionInput;
    reportVersionInput.m_Opcode = UVP_OPCODE::REPORT_VERSION;

    if (opExecutor.execute(reportVersionInput) != UVP_ERRORCODE::SUCCESS)
    {
        throw std::runtime_error("Report version op failed");
    }

    UvpMessageT* pVersionMessageBase = opExecutor.getLastMessage(UvpMessageT::MESSAGE_CODE::VERSION);
    if (!pVersionMessageBase)
    {
        throw std::runtime_error("Expected Version message not found");
    }

    UvpVersionMessageT* pVersionMessage = static_cast<UvpVersionMessageT*>(pVersionMessageBase);

    std::cout << "UVP core info:\n";
    std::cout << "Version: " << pVersionMessage->m_VersionMajor << "." << pVersionMessage->m_VersionMinor << "." << pVersionMessage->m_VersionPatch << "\n";
    std::cout << "Available packing devices in the system:\n";

    for (auto& devDesc : pVersionMessage->m_DeviceDescArray)
    {
        std::cout << "ID: " << devDesc.m_Id.c_str() << ", NAME: " << devDesc.m_Name.c_str() << ", SUPPORTED: " << devDesc.m_Supported << "\n";
    }

    //FbxUvWrapper fbxWrapper(pFbxFilePath, pMeshName, uvpInput.m_UvData);
    //
    //if (opExecutor.execute(uvpInput) != UVP_ERRORCODE::SUCCESS)
    //{
    //    ErrorLogger::LOG_ERROR("Packing operation failed!");
    //    return false;
    //}
    //
    //if (pOutFilePath)
    //{
    //    if (!opExecutor.getLastMessage(UvpMessageT::MESSAGE_CODE::ISLANDS) || !opExecutor.getLastMessage(UvpMessageT::MESSAGE_CODE::PACK_SOLUTION))
    //    {
    //        throw std::runtime_error("Expected UVP messages not found");
    //    }
    //
    //    const UvpIslandsMessageT* pIslandsMsg = static_cast<const UvpIslandsMessageT*>(opExecutor.getLastMessage(UvpMessageT::MESSAGE_CODE::ISLANDS));
    //    const UvpPackSolutionMessageT* pPackSolutionMsg = static_cast<const UvpPackSolutionMessageT*>(opExecutor.getLastMessage(UvpMessageT::MESSAGE_CODE::PACK_SOLUTION));
    //
    //    fbxWrapper.applyPackResult(pIslandsMsg, pPackSolutionMsg);
    //    fbxWrapper.saveToFile(pOutFilePath);
    //}

	return false;
}

bool TextureUnpacker::PerformTextureUnpack(geometry::Image& im, geometry::TriangleMesh& mesh, bool debug_info)
{
    UvpOperationInputT uvpInput;

    uvpInput.m_pDeviceId = "cpu";
    uvpInput.m_RenderResult = true;
    uvpInput.m_RenderInvalidIslands = true;
    uvpInput.m_RealtimeSolution = true;
    uvpInput.m_Benchmark = true;
    uvpInput.m_Opcode = UVP_OPCODE::PACK;

    UvpOpExecutorT opExecutor(debug_info);

    UvpOperationInputT reportVersionInput;
    reportVersionInput.m_Opcode = UVP_OPCODE::REPORT_VERSION;

    if (opExecutor.execute(reportVersionInput) != UVP_ERRORCODE::SUCCESS)
    {
        ErrorLogger::LOG_ERROR("Report version op failed");
    }

    UvpMessageT* pVersionMessageBase = opExecutor.getLastMessage(UvpMessageT::MESSAGE_CODE::VERSION);
    if (!pVersionMessageBase)
    {
        ErrorLogger::LOG_ERROR("Expected Version message not found");
    }

    UvpVersionMessageT* pVersionMessage = static_cast<UvpVersionMessageT*>(pVersionMessageBase);

    std::cout << "UVP core info:\n";
    std::cout << "Version: " << pVersionMessage->m_VersionMajor << "." << pVersionMessage->m_VersionMinor << "." << pVersionMessage->m_VersionPatch << "\n";
    std::cout << "Available packing devices in the system:\n";

    for (auto& devDesc : pVersionMessage->m_DeviceDescArray)
    {
        std::cout << "ID: " << devDesc.m_Id.c_str() << ", NAME: " << devDesc.m_Name.c_str() << ", SUPPORTED: " << devDesc.m_Supported << "\n";
    }

    return false;
}
