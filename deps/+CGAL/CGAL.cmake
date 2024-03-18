add_cmake_project(
    CGAL
    GIT_REPOSITORY https://github.com/CGAL/cgal.git
    GIT_TAG        e4c9cfd539729d0db5f64bed303a4816719d0792
)

include(GNUInstallDirs)

set(DEP_CGAL_DEPENDS Boost GMP MPFR)
