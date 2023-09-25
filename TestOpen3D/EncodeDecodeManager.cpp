#include "EncodeDecodeManager.h"

void EncDec::EncodeDecodeManager::split_int(int& shift_x, int& shift_y, int index)
{
    shift_x |= (((index & (1 << 0)) > 0) << 0);
    shift_y |= (((index & (1 << 1)) > 0) << 0);
    shift_x |= (((index & (1 << 2)) > 0) << 1);
    shift_y |= (((index & (1 << 3)) > 0) << 1);
    shift_x |= (((index & (1 << 4)) > 0) << 2);
    shift_y |= (((index & (1 << 5)) > 0) << 2);
    shift_x |= (((index & (1 << 6)) > 0) << 3);
    shift_y |= (((index & (1 << 7)) > 0) << 3);
    shift_x |= (((index & (1 << 8)) > 0) << 4);
    shift_y |= (((index & (1 << 9)) > 0) << 4);
    shift_x |= (((index & (1 << 10)) > 0) << 5);
    shift_y |= (((index & (1 << 11)) > 0) << 5);
    shift_x |= (((index & (1 << 12)) > 0) << 6);
    shift_y |= (((index & (1 << 13)) > 0) << 6);
    shift_x |= (((index & (1 << 14)) > 0) << 7);
    shift_y |= (((index & (1 << 15)) > 0) << 7);
    shift_x |= (((index & (1 << 16)) > 0) << 8);
    shift_y |= (((index & (1 << 17)) > 0) << 8);
    shift_x |= (((index & (1 << 18)) > 0) << 9);
    shift_y |= (((index & (1 << 19)) > 0) << 9);
    shift_x |= (((index & (1 << 20)) > 0) << 10);
    shift_y |= (((index & (1 << 21)) > 0) << 10);
    shift_x |= (((index & (1 << 22)) > 0) << 11);
    shift_y |= (((index & (1 << 23)) > 0) << 11);
    shift_x |= (((index & (1 << 24)) > 0) << 12);
    shift_y |= (((index & (1 << 25)) > 0) << 12);
    shift_x |= (((index & (1 << 26)) > 0) << 13);
    shift_y |= (((index & (1 << 27)) > 0) << 13);
    shift_x |= (((index & (1 << 28)) > 0) << 14);
    shift_y |= (((index & (1 << 29)) > 0) << 14);
    shift_x |= (((index & (1 << 30)) > 0) << 15);
    shift_y |= (((index & (1 << 31)) > 0) << 15);
}

void EncDec::EncodeDecodeManager::combine_int(int shift_x, int shift_y, int& return_value)
{
    return_value |= ((shift_x & 1 << 0) > 0) << 0;
    return_value |= ((shift_y & 1 << 0) > 0) << 1;
    return_value |= ((shift_x & 1 << 1) > 0) << 2;
    return_value |= ((shift_y & 1 << 1) > 0) << 3;
    return_value |= ((shift_x & 1 << 2) > 0) << 4;
    return_value |= ((shift_y & 1 << 2) > 0) << 5;
    return_value |= ((shift_x & 1 << 3) > 0) << 6;
    return_value |= ((shift_y & 1 << 3) > 0) << 7;
    return_value |= ((shift_x & 1 << 4) > 0) << 8;
    return_value |= ((shift_y & 1 << 4) > 0) << 9;
    return_value |= ((shift_x & 1 << 5) > 0) << 10;
    return_value |= ((shift_y & 1 << 5) > 0) << 11;
    return_value |= ((shift_x & 1 << 6) > 0) << 12;
    return_value |= ((shift_y & 1 << 6) > 0) << 13;
    return_value |= ((shift_x & 1 << 7) > 0) << 14;
    return_value |= ((shift_y & 1 << 7) > 0) << 15;
    return_value |= ((shift_x & 1 << 8) > 0) << 16;
    return_value |= ((shift_y & 1 << 8) > 0) << 17;
    return_value |= ((shift_x & 1 << 9) > 0) << 18;
    return_value |= ((shift_y & 1 << 9) > 0) << 19;
    return_value |= ((shift_x & 1 << 10) > 0) << 20;
    return_value |= ((shift_y & 1 << 10) > 0) << 21;
    return_value |= ((shift_x & 1 << 11) > 0) << 22;
    return_value |= ((shift_y & 1 << 11) > 0) << 23;
    return_value |= ((shift_x & 1 << 12) > 0) << 24;
    return_value |= ((shift_y & 1 << 12) > 0) << 25;
    return_value |= ((shift_x & 1 << 13) > 0) << 26;
    return_value |= ((shift_y & 1 << 13) > 0) << 27;
    return_value |= ((shift_x & 1 << 14) > 0) << 28;
    return_value |= ((shift_y & 1 << 14) > 0) << 29;
    return_value |= ((shift_x & 1 << 15) > 0) << 30;
    return_value |= ((shift_y & 1 << 15) > 0) << 31;
}

std::shared_ptr<open3d::geometry::Image> EncDec::EncodeDecodeManager::DefaultEncodeOBJ(std::shared_ptr<open3d::geometry::TriangleMesh> mesh, int image_channels, int image_bytes_per_channel)
{
    auto to_return = std::make_shared<open3d::geometry::Image>();

    int ind_num = mesh->triangles_.size();
    int ind_size = ind_num * sizeof(Eigen::Vector3i);
    int vert_num = mesh->vertices_.size();
    int vert_size = vert_num * sizeof(Eigen::Vector3d);
    int uv_num = mesh->triangle_uvs_.size();
    int uv_size = uv_num * sizeof(Eigen::Vector2d);
    int normal_num = mesh->vertex_normals_.size();
    int normal_size = normal_num * sizeof(Eigen::Vector3d);

    std::cout << "Independant Stats: \n"
        << ind_num << "\n"
        << ind_size << "\n"
        << vert_num << "\n"
        << vert_size << "\n"
        << uv_num << "\n"
        << uv_size << "\n"
        << normal_num << "\n"
        << normal_size << std::endl;

    int pos_ind_num = 0;
    int pos_vert_num = pos_ind_num + sizeof(ind_num);
    int pos_uv_num = pos_vert_num + sizeof(vert_num);
    int pos_normal_num = pos_uv_num + sizeof(uv_num);

    int pos_indicies = pos_normal_num + sizeof(normal_num);
    int pos_vertices = pos_indicies + ind_size;
    int pos_uvs = pos_vertices + vert_size;
    int pos_normals = pos_uvs + uv_size;

    int endpoint = pos_normals + normal_size;

    std::cout << "Position Stats: \n"
        << pos_ind_num << "\n"
        << pos_vert_num << "\n"
        << pos_uv_num << "\n"
        << pos_normal_num << "\n"

        << pos_indicies << "\n"
        << pos_vertices << "\n"
        << pos_uvs << "\n"
        << pos_normals << std::endl;

    std::cout << "ENDPOINT: " << endpoint << std::endl;

    auto min_size = sqrt(endpoint / (image_bytes_per_channel * image_channels));

    int image_width = ceil(min_size) + 1;
    int image_height = image_width;

    std::cout << "IMAGE DIMENSIONS: " << image_width << ", " << image_height << std::endl;

    int data_size = image_width * image_height * image_bytes_per_channel * image_channels;

    std::cout << "Image size: " << data_size << std::endl;

    to_return->Prepare(image_width, image_height, image_channels, image_bytes_per_channel);

    //to_return->width_ = image_width;
    //to_return->height_ = image_height;
    //
    //to_return->bytes_per_channel_ = image_bytes_per_channel;
    //to_return->num_of_channels_ = image_channels;
    //
    //to_return->data_.resize(data_size);
    
    memcpy(to_return->data_.data() + pos_ind_num, &ind_num, sizeof(ind_num));
    memcpy(to_return->data_.data() + pos_vert_num, &vert_num, sizeof(vert_num));
    memcpy(to_return->data_.data() + pos_uv_num, &uv_num, sizeof(uv_num));
    memcpy(to_return->data_.data() + pos_normal_num, &normal_num, sizeof(normal_num));
    
    memcpy(to_return->data_.data() + pos_indicies, mesh->triangles_.data(), ind_size);
    memcpy(to_return->data_.data() + pos_vertices, mesh->vertices_.data(), vert_size);
    memcpy(to_return->data_.data() + pos_uvs, mesh->triangle_uvs_.data(), uv_size);
    memcpy(to_return->data_.data() + pos_normals, mesh->vertex_normals_.data(), normal_size);

    return to_return;
}

std::shared_ptr<open3d::geometry::TriangleMesh> EncDec::EncodeDecodeManager::DefaultDecodeOBJ(std::shared_ptr<open3d::geometry::Image> image)
{
    auto to_return = std::make_shared<open3d::geometry::TriangleMesh>();

    int loc = 0;

    int iter_size = sizeof(int);


    int ind_num = 0;
    memcpy(&ind_num, image->data_.data() + loc, iter_size);
    loc += iter_size;

    int vert_num = 0;
    memcpy(&vert_num, image->data_.data() + loc, iter_size);
    loc += iter_size;

    int uv_num = 0;
    memcpy(&uv_num, image->data_.data() + loc, iter_size);
    loc += iter_size;

    int normal_num = 0;
    memcpy(&normal_num, image->data_.data() + loc, iter_size);
    loc += iter_size;

    int ind_size = ind_num * sizeof(Eigen::Vector3i);
    to_return->triangles_.resize(ind_num);
    std::cout << to_return->triangles_.size() << " indices - " << ind_size << " bytes allocated" << std::endl;
    memcpy(to_return->triangles_.data(), image->data_.data() + loc, ind_size);
    loc += ind_size;

    int vert_size = vert_num * sizeof(Eigen::Vector3d);
    to_return->vertices_.resize(vert_num);
    std::cout << to_return->vertices_.size() << " vertices - " << vert_size << " bytes allocated" << std::endl;
    memcpy(to_return->vertices_.data(), image->data_.data() + loc, vert_size);
    loc += vert_size;

    int uv_size = uv_num * sizeof(Eigen::Vector2d);
    to_return->triangle_uvs_.resize(uv_num);
    std::cout << to_return->triangle_uvs_.size() << " uvs - " << uv_size << " bytes allocated" << std::endl;
    memcpy(to_return->triangle_uvs_.data(), image->data_.data() + loc, uv_size);
    loc += uv_size;

    int normal_size = normal_num * sizeof(Eigen::Vector3d);
    to_return->vertex_normals_.resize(normal_num);
    std::cout << to_return->vertex_normals_.size() << " normals - " << normal_size << " bytes allocated" << std::endl;
    memcpy(to_return->vertex_normals_.data(), image->data_.data() + loc, normal_size);
    loc += normal_size;


    std::cout << "Independant Stats: \n"
        << ind_num << "\n"
        << ind_size << "\n"
        << vert_num << "\n"
        << vert_size << "\n"
        << uv_num << "\n"
        << uv_size << "\n"
        << normal_num << "\n"
        << normal_size << std::endl;


    return to_return;

}

void EncDec::EncodeDecodeManager::write_morton_single_simplified(byte* _dest, const byte* _src, int data_count, int bytes_per_line, int bytes_per_pixel)
{
    int max_step = data_count / bytes_per_pixel;

    if (max_step > 0)
    {
        for (int i = 0; i < max_step; ++i)
        {
            int shift_x = 0;
            int shift_y = 0;

            split_int(shift_x, shift_y, i);

            int position = shift_y * bytes_per_line + shift_x * bytes_per_pixel;

            memcpy(_dest + position, _src + i * bytes_per_pixel, bytes_per_pixel);
        }
    }

    int data_stop = max_step * bytes_per_pixel;
    int remainder_data = data_count - data_stop;

    if (remainder_data > 0)
    {
        int final_x = 0;
        int final_y = 0;

        split_int(final_x, final_y, max_step);

        int position = final_y * bytes_per_line + final_x * bytes_per_pixel;

        memcpy(_dest + position, _src + data_stop, remainder_data);
    }
}

std::shared_ptr<open3d::geometry::Image> EncDec::EncodeDecodeManager::MortonEncodeOBJ(std::shared_ptr<open3d::geometry::TriangleMesh> mesh, int image_channels, int image_bytes_per_channel)
{
    auto to_return = std::make_shared<open3d::geometry::Image>();

    int ind_num = mesh->triangles_.size();
    int ind_size = ind_num * sizeof(Eigen::Vector3i);
    int vert_num = mesh->vertices_.size();
    int vert_size = vert_num * sizeof(Eigen::Vector3d);
    int uv_num = mesh->triangle_uvs_.size();
    int uv_size = uv_num * sizeof(Eigen::Vector2d);
    int normal_num = mesh->vertex_normals_.size();
    int normal_size = normal_num * sizeof(Eigen::Vector3d);

    std::cout << "Independant Stats: \n"
        << ind_num << "\n"
        << ind_size << "\n"
        << vert_num << "\n"
        << vert_size << "\n"
        << uv_num << "\n"
        << uv_size << "\n"
        << normal_num << "\n"
        << normal_size << std::endl;

    int pos_ind_num = 0;
    int pos_vert_num = pos_ind_num + sizeof(ind_num);
    int pos_uv_num = pos_vert_num + sizeof(vert_num);
    int pos_normal_num = pos_uv_num + sizeof(uv_num);

    int pos_indicies = pos_normal_num + sizeof(normal_num);
    int pos_vertices = pos_indicies + ind_size;
    int pos_uvs = pos_vertices + vert_size;
    int pos_normals = pos_uvs + uv_size;

    int data_count = pos_normals + normal_size;

    int endpoint = data_count + sizeof(data_count);

    std::cout << "Position Stats: \n"
        << pos_ind_num << "\n"
        << pos_vert_num << "\n"
        << pos_uv_num << "\n"
        << pos_normal_num << "\n"

        << pos_indicies << "\n"
        << pos_vertices << "\n"
        << pos_uvs << "\n"
        << pos_normals << std::endl;

    std::cout << "ENDPOINT: " << endpoint << std::endl;

    auto min_size = sqrt(endpoint / (image_bytes_per_channel * image_channels));

    int image_width = pow(2, ceil(log2(min_size))) + 1;
    int image_height = image_width;

    int data_size = image_width * image_height * image_bytes_per_channel * image_channels;

    std::cout << "Image size: " << data_size << std::endl;

    if (data_size < endpoint)
    {
        std::cout << "ERROR: data was too large for image size!" << std::endl;
    }
    else
    {
        to_return->Prepare(image_width, image_height, image_channels, image_bytes_per_channel);

        int bytes_per_pixel = image_channels * image_bytes_per_channel;

        int bytes_per_line = image_width * bytes_per_pixel;

        std::vector<byte> data_dump;

        data_dump.resize(endpoint);

        int loc = 0;
        int iter_size = sizeof(int);

        data_count += iter_size;

        memcpy(data_dump.data() + loc, &data_count, iter_size);
        loc += iter_size;
        memcpy(data_dump.data() + loc, &ind_num, iter_size);
        loc += iter_size;
        memcpy(data_dump.data() + loc, &vert_num, iter_size);
        loc += iter_size;
        memcpy(data_dump.data() + loc, &uv_num, iter_size);
        loc += iter_size;
        memcpy(data_dump.data() + loc, &normal_num, iter_size);
        loc += iter_size;

        memcpy(data_dump.data() + loc, mesh->triangles_.data(), ind_size);
        loc += ind_size;
        memcpy(data_dump.data() + loc, mesh->vertices_.data(), vert_size);
        loc += vert_size;
        memcpy(data_dump.data() + loc, mesh->triangle_uvs_.data(), uv_size);
        loc += uv_size;
        memcpy(data_dump.data() + loc, mesh->vertex_normals_.data(), normal_size);
        loc += normal_size;

        to_return->data_.resize(data_size);

        int max_step = endpoint / bytes_per_pixel;

        write_morton_single_simplified(to_return->data_.data(), data_dump.data(), data_dump.size(), bytes_per_line, bytes_per_pixel);

        //std::cout << "Starting ind_num at " << current_byte << std::endl;
        //current_byte = morton_order_single(current_byte, to_return, (byte*)&ind_num, sizeof(ind_num), bytes_per_line, bytes_per_pixel);
        //std::cout << "Starting vert_num at " << current_byte << std::endl;
        //current_byte = morton_order_single(current_byte, to_return, (byte*)&vert_num, sizeof(vert_num), bytes_per_line, bytes_per_pixel);
        //std::cout << "Starting uv_num at " << current_byte << std::endl;
        //current_byte = morton_order_single(current_byte, to_return, (byte*)&uv_num, sizeof(uv_num), bytes_per_line, bytes_per_pixel);
        //std::cout << "Starting normal_num at " << current_byte << std::endl;
        //current_byte = morton_order_single(current_byte, to_return, (byte*)&normal_num, sizeof(normal_num), bytes_per_line, bytes_per_pixel);
        //
        //std::cout << "Starting indicies at " << current_byte << std::endl;
        //current_byte = morton_order_single(current_byte, to_return, (byte*)mesh->triangles_.data(), ind_size, bytes_per_line, bytes_per_pixel);
        //std::cout << "Starting vertices at " << current_byte << std::endl;
        //current_byte = morton_order_single(current_byte, to_return, (byte*)mesh->vertices_.data(), vert_size, bytes_per_line, bytes_per_pixel);
        //std::cout << "Starting uvs at " << current_byte << std::endl;
        //current_byte = morton_order_single(current_byte, to_return, (byte*)mesh->triangle_uvs_.data(), uv_size, bytes_per_line, bytes_per_pixel);
        //std::cout << "Starting normals at " << current_byte << std::endl;
        //current_byte = morton_order_single(current_byte, to_return, (byte*)mesh->vertex_normals_.data(), normal_size, bytes_per_line, bytes_per_pixel);
    }

    return to_return;
}

void EncDec::EncodeDecodeManager::read_morton_single_simplified(byte* _dest, const byte* _src, int byte_count, int bytes_per_line, int bytes_per_pixel)
{
    int max_step = byte_count / bytes_per_pixel;

    if (max_step > 0)
    {
        for (int i = 0; i < max_step; ++i)
        {
            int shift_x = 0;
            int shift_y = 0;

            split_int(shift_x, shift_y, i);

            int position = shift_y * bytes_per_line + shift_x * bytes_per_pixel;

            memcpy(_dest + i * bytes_per_pixel, _src + position, bytes_per_pixel);
        }
    }

    int data_stop = max_step * bytes_per_pixel;
    int remainder_data = byte_count - data_stop;

    if (remainder_data > 0)
    {
        int final_x = 0;
        int final_y = 0;

        split_int(final_x, final_y, max_step);

        int position = final_y * bytes_per_line + final_x * bytes_per_pixel;

        memcpy(_dest + data_stop, _src + position, remainder_data);
    }
}

std::shared_ptr<open3d::geometry::TriangleMesh> EncDec::EncodeDecodeManager::MortonDecodeOBJ(std::shared_ptr<open3d::geometry::Image> image)
{
    auto to_return = std::make_shared<open3d::geometry::TriangleMesh>();

    int iter_size = sizeof(int);

    int data_count = 0;

    int image_width = image->width_;
    int image_height = image->height_;

    int bytes_per_line = image->BytesPerLine();
    int channel_num = image->num_of_channels_;
    int bytes_per_pixel = image->bytes_per_channel_ * channel_num;

    std::vector<byte> data_dump;

    read_morton_single_simplified((byte*)&data_count, image->data_.data(), iter_size, bytes_per_line, bytes_per_pixel);

    data_dump.resize(data_count);

    read_morton_single_simplified(data_dump.data(), image->data_.data(), data_count, bytes_per_line, bytes_per_pixel);

    int loc = iter_size;

    int ind_num = 0;
    memcpy(&ind_num, data_dump.data() + loc, iter_size);
    loc += iter_size;

    int vert_num = 0;
    memcpy(&vert_num, data_dump.data() + loc, iter_size);
    loc += iter_size;

    int uv_num = 0;
    memcpy(&uv_num, data_dump.data() + loc, iter_size);
    loc += iter_size;

    int normal_num = 0;
    memcpy(&normal_num, data_dump.data() + loc, iter_size);
    loc += iter_size;


    to_return->triangles_.resize(ind_num);
    memcpy(to_return->triangles_.data(), data_dump.data() + loc, ind_num * sizeof(Eigen::Vector3i));
    int ind_size = to_return->triangles_.size() * sizeof(Eigen::Vector3i);
    loc += ind_size;

    to_return->vertices_.resize(vert_num);
    memcpy(to_return->vertices_.data(), data_dump.data() + loc, vert_num * sizeof(Eigen::Vector3d));
    int vert_size = to_return->vertices_.size() * sizeof(Eigen::Vector3d);
    loc += vert_size;

    to_return->triangle_uvs_.resize(uv_num);
    memcpy(to_return->triangle_uvs_.data(), data_dump.data() + loc, uv_num * sizeof(Eigen::Vector2d));
    int uv_size = to_return->triangle_uvs_.size() * sizeof(Eigen::Vector2d);
    loc += uv_size;

    to_return->vertex_normals_.resize(normal_num);
    memcpy(to_return->vertex_normals_.data(), data_dump.data() + loc, normal_num * sizeof(Eigen::Vector3d));
    int normal_size = to_return->vertex_normals_.size() * sizeof(Eigen::Vector3d);
    loc += normal_size;

    return to_return;
}

bool EncDec::EncodeDecodeManager::FFT()
{
    

    return false;
}
