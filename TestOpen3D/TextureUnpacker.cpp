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

/// <summary>
/// Packs UVs into a confined space and produces an output texture map
/// </summary>
/// <param name="color_array">Array of reference images consisting of all relevant color textures</param>
/// <param name="mesh">Mesh with UVs to unpack</param>
/// <param name="outputImage">Output texture. Resize this to the desired height/width/channels/etc before sending it to this function.</param>
/// <param name="debug_info">Whether or not to debug additional information - highly inefficient, only use to debug</param>
/// <returns></returns>
bool TextureUnpacker::PerformTextureUnpack(std::vector<open3d::geometry::Image>* color_array, geometry::TriangleMesh* mesh, geometry::Image* outputImage, bool debug_info)
{
    UvpOperationInputT uvpInput;

    uvpInput.m_pDeviceId = "cpu";
    uvpInput.m_RenderResult = false;// true;
    uvpInput.m_RenderInvalidIslands = false;// true;
    uvpInput.m_RealtimeSolution = true;
    uvpInput.m_Benchmark = false; // true;
    uvpInput.m_Opcode = UVP_OPCODE::PACK;

    UvpOpExecutorT opExecutor(debug_info);

    UvpOperationInputT reportVersionInput;
    reportVersionInput.m_Opcode = UVP_OPCODE::REPORT_VERSION;

    //Confirm version
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

    //Create data to send to the algorithm
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

    //The algorithm
    auto return_val = opExecutor.execute(uvpInput);

    if (return_val != UVP_ERRORCODE::SUCCESS)
    {
        std::cout << "Error type: " << (int)return_val << std::endl;
        E_LOG("Packing operation failed", true);
        return false;
    }

    if (!opExecutor.getLastMessage(UvpMessageT::MESSAGE_CODE::ISLANDS) || !opExecutor.getLastMessage(UvpMessageT::MESSAGE_CODE::PACK_SOLUTION))
    {
        E_LOG("Expected UVP messages not found", true);
        return false;
    }
    
    const UvpIslandsMessageT* pIslandsMsg = static_cast<const UvpIslandsMessageT*>(opExecutor.getLastMessage(UvpMessageT::MESSAGE_CODE::ISLANDS));
    const UvpPackSolutionMessageT* pPackSolutionMsg = static_cast<const UvpPackSolutionMessageT*>(opExecutor.getLastMessage(UvpMessageT::MESSAGE_CODE::PACK_SOLUTION));
    
    std::vector<Eigen::Matrix4d> island_matrices;
    std::vector<int> uv_solution;

    uv_solution.resize(mesh->triangle_uvs_.size());

    double width_destination = (double)outputImage->width_;
    double height_destination = (double)outputImage->height_;

    int width_destination_int = outputImage->width_;
    int height_destination_int = outputImage->height_;

    double width_source = (double)color_array->at(0).width_;
    double height_source = (double)color_array->at(0).height_;

    double step = 1.0 / sqrt(2.0);

    std::vector<Eigen::Vector2d> baryc_source;
    std::vector<Eigen::Vector2d> baryc_destination;

    baryc_source.resize(3);
    baryc_destination.resize(3);

    //May be useless, delete later if so
    std::vector<float> image_weights;
    image_weights.resize(width_destination_int * height_destination_int, 0);

    //Re-apply UVs
    const auto& islands = pIslandsMsg->m_Islands;
    for (const UvpIslandPackSolutionT& islandSolution : pPackSolutionMsg->m_IslandSolutions)
    {
        const IdxArrayT& island = islands[islandSolution.m_IslandIdx];
        Eigen::Matrix4d solutionMatrix;
        islandSolutionToMatrix(islandSolution, solutionMatrix);

        for (int faceId : island)
        {
            const UvFaceT& face = m_FaceArray[faceId];

            int source_image = mesh->triangle_material_ids_[faceId];

            int v_num = 0;

            for (int vertIdx : face.m_Verts)
            {
                const UvVertT& origVert = m_VertArray[vertIdx];

                baryc_source[v_num].x() = origVert.m_UvCoords[0] * width_source;
                baryc_source[v_num].y() = origVert.m_UvCoords[1] * height_source;

                Eigen::Vector4d transformedUV = solutionMatrix * Eigen::Vector4d(origVert.m_UvCoords[0], origVert.m_UvCoords[1], 0.0, 1.0);

                mesh->triangle_uvs_[vertIdx].x() = transformedUV.x();
                mesh->triangle_uvs_[vertIdx].y() = transformedUV.y();
                uv_solution[vertIdx] = island_matrices.size();

                baryc_destination[v_num].x() = transformedUV.x() * width_destination;
                baryc_destination[v_num].y() = transformedUV.y() * height_destination;

                ++v_num;
            }

            Eigen::Vector2d baryc_dim_a = baryc_destination[0] - baryc_destination[1];
            Eigen::Vector2d baryc_dim_b = baryc_destination[0] - baryc_destination[2];
            Eigen::Vector2d baryc_dim_c = baryc_destination[1] - baryc_destination[2];

            double max_dist = std::max(std::max(sqrt(baryc_dim_a.dot(baryc_dim_a)), sqrt(baryc_dim_b.dot(baryc_dim_b))), sqrt(baryc_dim_c.dot(baryc_dim_c)));

            if (max_dist == 0) //Triangle has no area
            {
                continue;
            }

            double baryc_step_a = step / max_dist;
            double baryc_step_b = step / max_dist;

            double offset = 1;

            for (double baryc_alpha = (-offset) * baryc_step_a; baryc_alpha < 1.0 + offset * baryc_step_a; baryc_alpha += baryc_step_a)
            {
                for (double baryc_beta = (-offset) * baryc_step_b; baryc_beta < 1.0 - (baryc_alpha * offset) + offset * baryc_step_b; baryc_beta += baryc_step_b)
                {
                    double p0 = (1.0 - baryc_alpha - baryc_beta);
                    double p1 = baryc_alpha;
                    double p2 = baryc_beta;

                    Eigen::Vector2d pixel_source = baryc_source[0] * p0 + baryc_source[1] * p1 + baryc_source[2] * p2;
                    Eigen::Vector2d pixel_destination = baryc_destination[0] * p0 + baryc_destination[1] * p1 + baryc_destination[2] * p2;

                    double x_source_min = std::floor(pixel_source.x());
                    double x_source_max = x_source_min + 1;
                    double y_source_min = std::floor(pixel_source.y());
                    double y_source_max = y_source_min + 1;

                    double delta_x = x_source_max - pixel_source.x();
                    double delta_y = y_source_max - pixel_source.y();

                    int u_source_min = std::clamp(x_source_min, 0.0, width_source - 1.0);
                    int v_source_min = std::clamp(y_source_min, 0.0, height_source - 1.0);
                    int u_source_max = std::clamp(x_source_max, 0.0, width_source - 1.0);
                    int v_source_max = std::clamp(y_source_max, 0.0, height_source - 1.0);

                    int u_destination = std::clamp(pixel_destination.x(), 0.0, width_destination - 1.0);
                    int v_destination = std::clamp(pixel_destination.y(), 0.0, height_destination - 1.0);

                    //Invert the Y coords
                    v_source_min = height_source - v_source_min - 1;
                    v_source_max = height_source - v_source_max - 1;
                    v_destination = height_destination - v_destination - 1;

                    (*outputImage->PointerAt<uint8_t>(u_destination, v_destination, 0)) = (*color_array->at(source_image).PointerAt<uint8_t>(u_source_min, v_source_min, 0));
                    (*outputImage->PointerAt<uint8_t>(u_destination, v_destination, 1)) = (*color_array->at(source_image).PointerAt<uint8_t>(u_source_min, v_source_min, 1));
                    (*outputImage->PointerAt<uint8_t>(u_destination, v_destination, 2)) = (*color_array->at(source_image).PointerAt<uint8_t>(u_source_min, v_source_min, 2));

                    //One day the code below will work
                    
                    //(*outputImage->PointerAt<uint8_t>(u0, v0, 0)) = image_weights[v0 * (int)w + u0] * (*outputImage->PointerAt<uint8_t>(u0, v0, 0)) + deltaX * deltaY * (*color_array->at(source_image).PointerAt<uint8_t>(u2, v2, 0));
                    //(*outputImage->PointerAt<uint8_t>(u0, v0, 1)) = image_weights[v0 * (int)w + u0] * (*outputImage->PointerAt<uint8_t>(u0, v0, 1)) + deltaX * deltaY * (*color_array->at(source_image).PointerAt<uint8_t>(u2, v2, 1));
                    //(*outputImage->PointerAt<uint8_t>(u0, v0, 2)) = image_weights[v0 * (int)w + u0] * (*outputImage->PointerAt<uint8_t>(u0, v0, 2)) + deltaX * deltaY * (*color_array->at(source_image).PointerAt<uint8_t>(u2, v2, 2));
                    //
                    //(*outputImage->PointerAt<uint8_t>(u0, v1, 0)) = image_weights[v1 * (int)w + u0] * (*outputImage->PointerAt<uint8_t>(u0, v1, 0)) + deltaX * (1.0 - deltaY) * (*color_array->at(source_image).PointerAt<uint8_t>(u2, v2, 0));
                    //(*outputImage->PointerAt<uint8_t>(u0, v1, 1)) = image_weights[v1 * (int)w + u0] * (*outputImage->PointerAt<uint8_t>(u0, v1, 1)) + deltaX * (1.0 - deltaY) * (*color_array->at(source_image).PointerAt<uint8_t>(u2, v2, 1));
                    //(*outputImage->PointerAt<uint8_t>(u0, v1, 2)) = image_weights[v1 * (int)w + u0] * (*outputImage->PointerAt<uint8_t>(u0, v1, 2)) + deltaX * (1.0 - deltaY) * (*color_array->at(source_image).PointerAt<uint8_t>(u2, v2, 2));
                    //
                    //(*outputImage->PointerAt<uint8_t>(u1, v0, 0)) = image_weights[v0 * (int)w + u1] * (*outputImage->PointerAt<uint8_t>(u1, v0, 0)) + (1.0 - deltaX) * deltaY * (*color_array->at(source_image).PointerAt<uint8_t>(u2, v2, 0));
                    //(*outputImage->PointerAt<uint8_t>(u1, v0, 1)) = image_weights[v0 * (int)w + u1] * (*outputImage->PointerAt<uint8_t>(u1, v0, 1)) + (1.0 - deltaX) * deltaY * (*color_array->at(source_image).PointerAt<uint8_t>(u2, v2, 1));
                    //(*outputImage->PointerAt<uint8_t>(u1, v0, 2)) = image_weights[v0 * (int)w + u1] * (*outputImage->PointerAt<uint8_t>(u1, v0, 2)) + (1.0 - deltaX) * deltaY * (*color_array->at(source_image).PointerAt<uint8_t>(u2, v2, 2));
                    //
                    //(*outputImage->PointerAt<uint8_t>(u1, v1, 0)) = image_weights[v1 * (int)w + u1] * (*outputImage->PointerAt<uint8_t>(u1, v1, 0)) + (1.0 - deltaX) * (1.0 - deltaY) * (*color_array->at(source_image).PointerAt<uint8_t>(u2, v2, 0));
                    //(*outputImage->PointerAt<uint8_t>(u1, v1, 1)) = image_weights[v1 * (int)w + u1] * (*outputImage->PointerAt<uint8_t>(u1, v1, 1)) + (1.0 - deltaX) * (1.0 - deltaY) * (*color_array->at(source_image).PointerAt<uint8_t>(u2, v2, 1));
                    //(*outputImage->PointerAt<uint8_t>(u1, v1, 2)) = image_weights[v1 * (int)w + u1] * (*outputImage->PointerAt<uint8_t>(u1, v1, 2)) + (1.0 - deltaX) * (1.0 - deltaY) * (*color_array->at(source_image).PointerAt<uint8_t>(u2, v2, 2));
                    //
                    //image_weights[v0 * (int)w + u0] += deltaX * deltaY;
                    //image_weights[v1 * (int)w + u0] += deltaX * (1.0 - deltaY);
                    //image_weights[v0 * (int)w + u1] += (1.0 - deltaX) * deltaY;
                    //image_weights[v1 * (int)w + u1] += (1.0 - deltaX) * (1.0 - deltaY);
                    //
                    //(*outputImage->PointerAt<uint8_t>(u0, v0, 0)) /= (image_weights[v0 * (int)w + u0] + (image_weights[v0 * (int)w + u0] == 0));
                    //(*outputImage->PointerAt<uint8_t>(u0, v0, 1)) /= (image_weights[v0 * (int)w + u0] + (image_weights[v0 * (int)w + u0] == 0));
                    //(*outputImage->PointerAt<uint8_t>(u0, v0, 2)) /= (image_weights[v0 * (int)w + u0] + (image_weights[v0 * (int)w + u0] == 0));
                    //
                    //(*outputImage->PointerAt<uint8_t>(u0, v1, 0)) /= (image_weights[v1 * (int)w + u0] + (image_weights[v1 * (int)w + u0] == 0));
                    //(*outputImage->PointerAt<uint8_t>(u0, v1, 1)) /= (image_weights[v1 * (int)w + u0] + (image_weights[v1 * (int)w + u0] == 0));
                    //(*outputImage->PointerAt<uint8_t>(u0, v1, 2)) /= (image_weights[v1 * (int)w + u0] + (image_weights[v1 * (int)w + u0] == 0));
                    //
                    //(*outputImage->PointerAt<uint8_t>(u1, v0, 0)) /= (image_weights[v0 * (int)w + u1] + (image_weights[v0 * (int)w + u1] == 0));
                    //(*outputImage->PointerAt<uint8_t>(u1, v0, 1)) /= (image_weights[v0 * (int)w + u1] + (image_weights[v0 * (int)w + u1] == 0));
                    //(*outputImage->PointerAt<uint8_t>(u1, v0, 2)) /= (image_weights[v0 * (int)w + u1] + (image_weights[v0 * (int)w + u1] == 0));
                    //
                    //(*outputImage->PointerAt<uint8_t>(u1, v1, 0)) /= (image_weights[v1 * (int)w + u1] + (image_weights[v1 * (int)w + u1] == 0));
                    //(*outputImage->PointerAt<uint8_t>(u1, v1, 1)) /= (image_weights[v1 * (int)w + u1] + (image_weights[v1 * (int)w + u1] == 0));
                    //(*outputImage->PointerAt<uint8_t>(u1, v1, 2)) /= (image_weights[v1 * (int)w + u1] + (image_weights[v1 * (int)w + u1] == 0));
                }
            }
        }

        island_matrices.push_back(solutionMatrix);
    }

    return true;
}
