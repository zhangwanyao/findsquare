#pragma once
//#define MESH_COMPARE
//#define WALL_CONVPT

struct IntPoint2D
{
    int x;
    int y;
};

struct FloatPoint2D
{
    float x;
    float y;
};

std::string GetMeshOpeningPath();

std::string GetSelectCeilingPath();

std::string GetUpdateJsonPath();

std::string GetMeshOutPath();

std::string GetSquaredOutPath();

int GetSegMode();

int GetDsMode();

int GetSquareMode();

bool IsSquareByAxis();

bool IsCustomizeSuqare();

int GetSquareHeight();

int GetSquareWidth();

int GetMinSquareOffset();

std::vector<IntPoint2D> GetPolypt_uv();

std::vector<FloatPoint2D> GetPolypt_xy();

std::string GetAxisEqnConfig();


