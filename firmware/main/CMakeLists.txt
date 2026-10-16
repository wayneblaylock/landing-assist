idf_component_register(SRCS "main.c" "webserver.c"
                       INCLUDE_DIRS "."
                       REQUIRES softap nvs_flash esp_http_server
                       EMBED_FILES "index.html")
