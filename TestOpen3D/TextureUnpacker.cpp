#include "TextureUnpacker.h"

bool TextureUnpacker::PackUV(geometry::Image& im, geometry::TriangleMesh& mesh, bool debug)
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

bool TextureUnpacker::PerformTextureUnpack(geometry::Image* im, geometry::TriangleMesh* mesh, bool debug_info)
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
        return false;
    }

    UvpMessageT* pVersionMessageBase = opExecutor.getLastMessage(UvpMessageT::MESSAGE_CODE::VERSION);
    if (!pVersionMessageBase)
    {
        ErrorLogger::LOG_ERROR("Expected Version message not found");
        return false;
    }

    UvpVersionMessageT* pVersionMessage = static_cast<UvpVersionMessageT*>(pVersionMessageBase);

    std::cout << "UVP core info:\n";
    std::cout << "Version: " << pVersionMessage->m_VersionMajor << "." << pVersionMessage->m_VersionMinor << "." << pVersionMessage->m_VersionPatch << "\n";
    std::cout << "Available packing devices in the system:\n";

    for (auto& devDesc : pVersionMessage->m_DeviceDescArray)
    {
        std::cout << "ID: " << devDesc.m_Id.c_str() << ", NAME: " << devDesc.m_Name.c_str() << ", SUPPORTED: " << devDesc.m_Supported << "\n";
    }

    int m_PolyVertexCount;

    std::vector<UvVertT> m_VertArray;
    std::vector<UvFaceT> m_FaceArray;

    std::unordered_map<UvVertT, int, UvVertHashT, UvVertEqualT> vertPointerMap;

    for (int i = 0; i < mesh->vertices_.size(); ++i)
    {
        UvVertT uvVert;

        Eigen::Vector2d uvs = mesh->triangle_uvs_[i];

        uvVert.m_UvCoords[0] = uvs[0];
        uvVert.m_UvCoords[1] = uvs[1];
        uvVert.m_ControlId = i;

        m_VertArray.emplace_back(uvVert);
    }

    for (int faceIdx = 0; faceIdx < mesh->triangles_.size(); faceIdx++)
    {
        int faceSize = 3;
        m_PolyVertexCount += faceSize;

        m_FaceArray.emplace_back(faceIdx);
        UvFaceT& face = m_FaceArray.back();

        auto& faceVerts = face.m_Verts;
        faceVerts.reserve(faceSize);

        for (int vertIdx = 0; vertIdx < faceSize; vertIdx++)
        {
            faceVerts.pushBack(mesh->triangles_[faceIdx][vertIdx]);
        }
    }

    uvpInput.m_UvData.m_FaceCount = m_FaceArray.size();
    uvpInput.m_UvData.m_pFaceArray = m_FaceArray.data();

    uvpInput.m_UvData.m_VertCount = m_VertArray.size();
    uvpInput.m_UvData.m_pVertArray = m_VertArray.data();

    std::cout << "Unpacking..." << std::endl;

    auto return_val = opExecutor.execute(uvpInput);

    if (return_val != UVP_ERRORCODE::SUCCESS)
    {
        std::cout << "Error type: " << (int)return_val << std::endl;
        E_LOG("Packing operation failed", true);
    }

    if (!opExecutor.getLastMessage(UvpMessageT::MESSAGE_CODE::ISLANDS) || !opExecutor.getLastMessage(UvpMessageT::MESSAGE_CODE::PACK_SOLUTION))
    {
        E_LOG("Expected UVP messages not found", true);
    }
    
    const UvpIslandsMessageT* pIslandsMsg = static_cast<const UvpIslandsMessageT*>(opExecutor.getLastMessage(UvpMessageT::MESSAGE_CODE::ISLANDS));
    const UvpPackSolutionMessageT* pPackSolutionMsg = static_cast<const UvpPackSolutionMessageT*>(opExecutor.getLastMessage(UvpMessageT::MESSAGE_CODE::PACK_SOLUTION));
    

    //REAPPLY UVS
    //std::vector<FbxVector2> transformedUvs(m_VertArray.size());
    //
    //// Initially copy the original UV coordinates.
    //for (int i = 0; i < m_VertArray.size(); i++)
    //{
    //    const auto& origVert = m_VertArray[i];
    //    transformedUvs[i] = FbxVector2(origVert.m_UvCoords[0], origVert.m_UvCoords[1]);
    //}
    //
    //// Transform UV coordinates accordingly
    const auto& islands = pIslandsMsg->m_Islands;
    for (const UvpIslandPackSolutionT& islandSolution : pPackSolutionMsg->m_IslandSolutions)
    {
        const IdxArrayT& island = islands[islandSolution.m_IslandIdx];
        Eigen::Matrix4d solutionMatrix;
        islandSolutionToMatrix(islandSolution, solutionMatrix);
    
        for (int faceId : island)
        {
            const UvFaceT& face = m_FaceArray[faceId];
    
            for (int vertIdx : face.m_Verts)
            {
                const UvVertT& origVert = m_VertArray[vertIdx];
                //vec4 inputUv = { origVert.m_UvCoords[0], origVert.m_UvCoords[1], 0.0, 1.0 };
                //vec4 transformedUv;

                Eigen::Vector4d transformedUV = solutionMatrix * Eigen::Vector4d(origVert.m_UvCoords[0], origVert.m_UvCoords[1], 0.0, 1.0);
    
                //mat4x4_mul_vec4(transformedUv, solutionMatrix, inputUv);
                //transformedUvs[vertIdx] = FbxVector2(transformedUv[0] / transformedUv[3], transformedUv[1] / transformedUv[3]);

                mesh->triangle_uvs_[vertIdx].x() = transformedUV.x();
                mesh->triangle_uvs_[vertIdx].y() = transformedUV.y();
            }
        }
    }
    //
    //const bool useIndex = m_pUvElement->GetReferenceMode() != FbxGeometryElement::eDirect;
    //
    //if (useIndex)
    //{
    //    m_pUvElement->GetIndexArray().Resize(m_PolyVertexCount);
    //    m_pUvElement->GetDirectArray().Resize(transformedUvs.size());
    //}
    //else
    //{
    //    m_pUvElement->GetDirectArray().Resize(m_PolyVertexCount);
    //}
    //
    //int polyIdxCounter = 0;
    //
    //for (const UvFaceT& uvFace : m_FaceArray)
    //{
    //    for (const int vertIdx : uvFace.m_Verts)
    //    {
    //        if (useIndex)
    //        {
    //            m_pUvElement->GetIndexArray().SetAt(polyIdxCounter, vertIdx);
    //            m_pUvElement->GetDirectArray().SetAt(vertIdx, transformedUvs[vertIdx]);
    //        }
    //        else
    //        {
    //            m_pUvElement->GetDirectArray().SetAt(polyIdxCounter, transformedUvs[vertIdx]);
    //        }
    //
    //        polyIdxCounter++;
    //    }
    //}
    //fbxWrapper.saveToFile(pOutFilePath);

    return true;
}
