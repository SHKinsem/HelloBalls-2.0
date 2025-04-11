#pragma once

#ifdef __cplusplus
extern "C" {
#endif

    void mp3_player_init(void);
    void music_ui(void);
    void graph_plotting_ui(int*);
    void init_chart_animation(void);
    void update_chart(float*, int);
    void update_label(char*);
    void display_heatmap(int*, int, int);
    void update_heatmap(void*, int, int);

#ifdef __cplusplus
}
#endif



