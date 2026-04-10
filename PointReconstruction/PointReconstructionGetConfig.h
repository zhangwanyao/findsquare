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

typedef enum eSquareStrategyMode_t
{
    SQUARE_STRATEGY_REGULARIZED = 0,   // 正则化找方：轴线优先，否则参考墙
    SQUARE_STRATEGY_FIT_MAX_AREA = 1,  // 非正则：尽量贴合墙面并最大化面积
}eSquareStrategyMode;

std::string GetMeshOpeningPath();

std::string GetSelectCeilingPath();

std::string GetUpdateJsonPath();

std::string GetMeshOutPath();

std::string GetSquaredOutPath();

int GetSegMode();

int GetDsMode();

int GetSquareMode();

int GetSquareStrategyMode();

bool IsRegularizedSquareMode();

bool IsSquareByAxis();

bool IsCustomizeSuqare();

int GetSquareHeight();

int GetSquareWidth();

int GetMinSquareOffset();

std::vector<IntPoint2D> GetPolypt_uv();

std::vector<FloatPoint2D> GetPolypt_xy();

std::string GetAxisEqnConfig();

