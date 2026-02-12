# web相关配置
set(WEB_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/web_src")
set(INDEX_HTML "${WEB_SRC_DIR}/index.html")
set(INDEX_HTML_GZ "${WEB_SRC_DIR}/index.html.gz")

# 注册组件
idf_component_register(
        SRCS "esp-satori-eye.c"
        "camera.c"
        "wifi.c"
        "web_server.c"
        "benchmark.c"
        "ai.cpp"
        INCLUDE_DIRS "."
        EMBED_FILES "${INDEX_HTML_GZ}"
)

# 在编译前自动把 index.html 压缩为 index.html.gz
add_custom_command(
        OUTPUT ${INDEX_HTML_GZ}
        COMMAND python -c "import gzip, shutil; f_in=open('${INDEX_HTML}', 'rb'); f_out=gzip.open('${INDEX_HTML_GZ}', 'wb'); shutil.copyfileobj(f_in, f_out); f_in.close(); f_out.close()"
        DEPENDS ${INDEX_HTML}
        COMMENT "Compressing web assets: index.html -> index.html.gz"
        VERBATIM
)

# 创建自定义目标
add_custom_target(compress_assets DEPENDS ${INDEX_HTML_GZ})

# 让本组件依赖压缩目标
add_dependencies(${COMPONENT_LIB} compress_assets)
