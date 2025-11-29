#include <pthread.h>
#include <stdint.h> // uint8_t를 사용하기 위해 추가 (1byte)
#include <stdio.h>
#include <stdlib.h> // random
#include <time.h>

#define SIMULATION_DONE 10000   // 시뮬레이션 횟수
#define MAX_PLANE_COUNT 1000000 // 최대 공존 가능 비행기 수
#define LANDING_Q_COUNT 8       // 착륙 큐 개수
#define TAKEOFF_Q_COUNT 5       // 이륙 큐 개수
#define RUNWAY_COUNT 5          // 활주로 개수
#define TAKEOFF_ONLY 2          // 이륙 전용 활주로 설정 (idx로 지정)
//-----------------------
#define TAKEOFF_ONLY_SECOND 4

//@ TAKEOFF_ONLY 여러개 설정법
// #define MAX_TAKEOFF_ONLY 3
// const int takeoff_only_rw[MAX_TAKEOFF_ONLY] = {1,3,5};
// const int takeoff_only_count = 3;

//@ 공간 복잡도 개선 사항
// todo: Plane을 Takeoff_Plane, Landing_Plane 으로 구분 + [중요] pool도 나눠야 함
// todo: >> 메모리를 아낄 수 있음 (fuel, consume: 8byte save)
// todo: >> 스레드 2개는 비효율적
// todo: uint16_t: 2byte, uint32_t: 4byte 메모리 낭비 조절

//@ 시간 복잡도 개선 사항
// todo: thread 세부 분할? > lock 적용 비효율 생각해야 함
// todo: tree?

// 4*3 + 1*2 = 14 >> 16바이트 정렬
// int만 사용하는 것 보다 16바이트 세이브 가능
//+ type을 없앨 수도 있음 (홀수: lsb=1, 짝수: lsb=0)
typedef struct Plane {
    int idx;       // 비행기 식별번호(착륙: 짝수, 이륙: 홀수)
    int fuel;      // 비행기 연료
    int entryTime; // 큐 진입 시간(통계)
    int consume;   // 연료 소모 속도
    int type;      // 착륙: 0, 이륙: 1 (idx를 이용한 비교X)
} Plane;

// Plane 구조체의 next보다는 Node 구조체를 따로 빼서 next를 하는게 논리적
typedef struct Node {
    Plane plane;
    struct Node *next;
} Node;

// 착륙 큐, 이륙 큐
typedef struct Queue {
    Node *head; // 삭제 수행
    Node *tail; // 삽입 수행
    int size;   // 로드 밸런싱
} Queue;

// 긴급 리스트 ()
typedef struct Stack {
    Node *top;            // LIFO pointer
    int size;             // 통계?
    pthread_mutex_t lock; // 긴급 리스트
} EmergencyStack;

// 스레드 할당 자원
typedef struct Thread_arg {
    Queue *q;
} Arg;

//// 스레드 공유 자원
Node pool[MAX_PLANE_COUNT]; // malloc의 연산 부하 해결
Node *freed_head = pool;    // 해제된 리스트의 헤드(가용 가능한 청크)
// pthread_mutex_t poolMutex = PTHREAD_MUTEX_INITIALIZER;

Queue landingQ[LANDING_Q_COUNT]; // 착륙 큐
Queue takeoffQ[TAKEOFF_Q_COUNT]; // 이륙 큐
EmergencyStack emergS;
////

//@ 매 시간 단위마다 집계하기 위한 변수
int g_total_emergency_plane_count = 0; //* 긴급 착륙을 시행한 모든 비행기의 수와 비율
int g_total_plane_count = 0;           //* 생성 비행기 수
int g_total_crashed_plane_count = 0;   //* 사고 당한 모든 비행기의 수와 비율

// next를 다음 주소와 연결해주는 작업 (리스트의 장점: 삭제 연산)
void init_pool(void) {
    // 마지막 idx직전까지 연결, next는 포인터: 주소를 연결
    for (int i = 0; i < MAX_PLANE_COUNT - 1; i++) {
        pool[i].next = &pool[i + 1];
    }
    // 마지막 idx는 next가 NULL이어야 함.
    pool[MAX_PLANE_COUNT - 1].next = NULL;
}

// LIFO 구조 노드 반환
Node *alloc_node(void) {
    // 가용 가능한 청크가 없는 경우(다 씀)
    if (freed_head == NULL) {
        printf("freed_head is NULL (FULL MEMORY)\n");
        return NULL;
    }

    // 하나씩 가져가며 pool은 줄어듦.
    Node *newNode = freed_head;
    freed_head = freed_head->next;

    // 배열에서 떼어내 연결리스트로 사용할 것이기 때문에 기존 연결을 끊어줘야 함.
    newNode->next = NULL;
    return newNode;
}

// LIFO 구조 노드 해제 및 재사용을 위한 연결
void free_node(Node *temp) {
    // 해제된 청크를 다시 사용 (LIFO)
    temp->next = freed_head;
    freed_head = temp;
}

// FIFO 구조 큐 초기화
void init_queue(Queue *queue) {
    queue->head = NULL;
    queue->tail = NULL;
    queue->size = 0;
}

// FIFO 구조 큐 삽입
void enqueue(Queue *queue, Node *temp) {
    if (queue->tail == NULL) {
        // tail에 아무것도 없는 경우
        queue->head = temp; //head 조정
        queue->tail = temp;
        queue->size++;
        return;
    }
    temp->next = NULL; // 기존 연결이 있을 수 있으니 해제
    queue->tail->next = temp;
    queue->tail = temp;
    queue->size++;
}

// FIFO 구조 큐 삭제
Node *dequeue(Queue *queue) {
    if (queue->head == NULL)
        return NULL; // head에 아무것도 없는 경우

    Node *node = queue->head;
    queue->head = queue->head->next;

    if (queue->head == NULL)
        queue->tail = NULL; // tail 조정
    queue->size--;

    return node;
}

// 가장 짧은 큐의 인덱스 반환 (비행기 생성 시 사용)
int get_shortest_queue_idx(Queue *q_addr, int q_size) {
    int min_q_idx = -1;
    int min_q_size = 1000000000;
    for (int i = 0; i < q_size; i++) {
        if (q_addr[i].size < min_q_size) {
            min_q_size = q_addr[i].size;
            min_q_idx = i;
        }
    }
    return min_q_idx;
}

// 이/착륙 비행기 생성 및 큐 삽입 & 생성 비행기 수 집계
int generate_planes(int entryTime) {
    static int land_idx = 2; // 착륙: 짝수 정수
    static int take_idx = 1; // 이륙: 홀수 정수

    int land_planes_cnt = rand() % 5; //0~3
    int take_planes_cnt = rand() % 5;

    g_total_plane_count += (land_planes_cnt + take_planes_cnt); // 생성 비행기 수 집계

    int landingQ_idx = get_shortest_queue_idx(landingQ, LANDING_Q_COUNT); // 짧은 큐 한 번 구해서 그냥 다 넣기 (비행기 수 적을 때)
    int takeoffQ_idx = get_shortest_queue_idx(takeoffQ, TAKEOFF_Q_COUNT);

    // 착륙 비행기 정보 기입
    for (int i = 0; i < land_planes_cnt; i++) {
        Node *newNode = alloc_node(); // Node 할당
        newNode->plane.idx = land_idx;
        newNode->plane.fuel = rand() % 49 + 20;  // 20~68
        newNode->plane.entryTime = entryTime;    // 생성 시점(통계)
        newNode->plane.consume = rand() % 3 + 1; // 1~3: 0이 되면 안됨
        newNode->plane.type = 0;                 // 착륙: 0

        // int landingQ_idx = get_shortest_queue_idx(landingQ, LANDING_Q_COUNT); // 연산 수 증가
        land_idx += 2;
        enqueue(&landingQ[landingQ_idx], newNode); // 착륙 큐 삽입
    }
    //이륙 비행기 정보 기입
    for (int i = 0; i < take_planes_cnt; i++) {
        Node *newNode = alloc_node();
        newNode->plane.idx = take_idx;
        newNode->plane.entryTime = entryTime;
        newNode->plane.type = 1; //이륙: 1

        // int takeoffQ_idx = get_shortest_queue_idx(takeoffQ, TAKEOFF_Q_COUNT); // 연산 수 증가
        take_idx += 2;

        enqueue(&takeoffQ[takeoffQ_idx], newNode); // 이륙 큐 삽입
    }
}

// 스택 초기화
void init_emergency_stack(EmergencyStack *s) {
    s->top = NULL;
    s->size = 0;
    pthread_mutex_init(&s->lock, NULL); // lock 초기화: NULL (default)
}

// 스택 push (lock 필요)
void push_emergency(EmergencyStack *s, Node *emerg) {
    pthread_mutex_lock(&s->lock); // lock

    // 사실상 LIFO 구조의 연결리스트
    emerg->next = s->top; // 긴급한 비행기끼리 연결
    s->top = emerg;
    s->size++;

    pthread_mutex_unlock(&s->lock); // unlock
}

// 스택 전체 리스트 pop (하나씩 빼 줄 필요X -> lock 필요X)
Node *pop_all_emergency(EmergencyStack *s) {

    if (s->top == NULL)
        return NULL; // 긴급 착륙 비행기가 없던 경우

    // EmergencyStack *emerg_all_copy = s; // 동일한 객체를 가리켜 의미X
    Node *emerg_head = s->top;

    // 다음 tick을 위한 초기화
    s->top = NULL;
    s->size = 0;
    pthread_mutex_init(&s->lock, NULL);

    return emerg_head;
}

//// 스레드 함수 (go_..)
// 연료 감소 및 <0 도달 감지 + 긴급 리스트 연결 수행
void *go_fuel_dec_and_check(void *arg) {
    Arg *src = (Arg *)arg; // 스레드 인자 형변환
    Queue *q = src->q;     // 스레드가 들고온 도착 큐

    Node *prev = NULL;
    Node *curr = q->head;

    // dec_and_check
    while (curr != NULL) {
        curr->plane.fuel -= curr->plane.consume;

        //연료가 부족한 경우
        if (curr->plane.fuel <= 0) {
            Node *emergency = curr; // next가 변경되기 전에 복사

            //@ unlink
            // 맨 앞인 경우: 도착 큐 head 조정
            if (prev == NULL) {
                q->head = curr->next;
            }
            else {
                prev->next = curr->next;
            }
            // 맨 마지막인 경우: 도착 큐 tail 조정
            if (curr == q->tail) {
                q->tail = prev;
            }
            q->size--;
            curr = curr->next; // curr 재조정 (조건 재탐색)

            //@ link
            // 긴급 스택에 추가 (해당 주소의 next가 변경되므로 마지막에..)
            push_emergency(&emergS, emergency);
        }
        else {
            prev = curr;
            curr = curr->next;
        }
    }
}

// 잔여 활주로 수 반환: >0, 0
int is_there_remain_runway(int *rw_used) {
    int remainRW = 0;
    for (int i = 0; i < RUNWAY_COUNT; i++) {
        // 점유된 활주로가 있다면
        if (rw_used[i] == 1)
            continue;
        else
            remainRW++;
    }
    return remainRW;
}

// 각 역할 큐 전체 사이즈 참조 비교 (call by ref: 배열 반환이 안되네..)
void get_total_queue_size(Queue *landQ, Queue *takeQ,
                          int *l_total_landing_queue_size, int *l_total_takeoff_queue_size) {
    for (int i = 0; i < LANDING_Q_COUNT; i++)
        *l_total_landing_queue_size += landQ[i].size;
    for (int i = 0; i < TAKEOFF_Q_COUNT; i++)
        *l_total_takeoff_queue_size += takeQ[i].size;
}

// 가장 긴 큐의 인덱스 반환 (이륙 시 사용)
int get_longest_queue_idx(Queue *q_addr, int q_size) {
    int max_q_idx = -1;
    int max_q_size = -1;
    for (int i = 0; i < q_size; i++) {
        if (q_addr[i].size > max_q_size) {
            max_q_idx = i;
            max_q_size = q_addr[i].size;
        }
    }
    return max_q_idx;
}

// 착륙 처리 중 이륙 전용 활주로를 만난 경우 수행
void takeoff_process(int target_rw_idx, int tick,
                     int *l_total_takeoff_latency, int *rw_used,
                     int *l_total_takeoff_queue_size) {
    int takeoffQ_idx = get_longest_queue_idx(takeoffQ, TAKEOFF_Q_COUNT); //전역
    Node *takeoff = dequeue(&takeoffQ[takeoffQ_idx]);                    // 전역
    if (takeoff == NULL)
        return;

    // 부모(for문)의 지역 변수들에 바로 접근
    *l_total_takeoff_latency += (tick - takeoff->plane.entryTime);
    rw_used[target_rw_idx] = 1;
    l_total_takeoff_queue_size--;

    printf("[TAKEOFF][FROM LANDING] ID: %d, RW: %d, Type: %d\n",
           takeoff->plane.idx, target_rw_idx + 1, takeoff->plane.type);

    free_node(takeoff);
}

/////////////////// main
int main(void) {
    // 프로그램 시작하자마자 버퍼링 끄기
    setbuf(stdout, NULL);

    srand(time(NULL));
    // 풀 초기화
    init_pool();
    // 큐 초기화
    for (int i = 0; i < LANDING_Q_COUNT; i++)
        init_queue(&landingQ[i]);
    for (int i = 0; i < TAKEOFF_Q_COUNT; i++)
        init_queue(&takeoffQ[i]);
    // 긴급 스택 초기화
    init_emergency_stack(&emergS);

    //// simulation run
    // 틱 마다 한 작업만 수행 (활주로 마다)
    for (int tick = 1; tick <= SIMULATION_DONE; tick++) {

        int l_total_landing_latency = 0;     // 평균 착륙 대기시간 집계용
        int l_total_landing_plane_count = 0; // 평균 착륙 대기시간 집계용

        int l_total_takeoff_latency = 0;     // 평균 이륙 대기시간 집계용
        int l_total_takeoff_plane_count = 0; // 평균 이륙 대기시간 집계용

        int l_total_landing_remaining = 0; // 평균 남은 제한 시간 집계용

        generate_planes(tick); // 0~3대 비행기 이/착륙 큐 삽입, tick: entryTime

        // 비행기 삽입 후 연산
        int l_total_landing_queue_size = 0;
        int l_total_takeoff_queue_size = 0;
        get_total_queue_size(landingQ, takeoffQ,
                             &l_total_landing_queue_size, &l_total_takeoff_queue_size);

        int rw_used[RUNWAY_COUNT] = {0}; // 활주로 초기화 + used: 1

        pthread_t tid[LANDING_Q_COUNT]; // thread id
        Arg arg[LANDING_Q_COUNT];       // thread data

        // thread 생성 및 정보 저장 후 수행
        //// 연료 감소 & <0 도달 감지 & EmergencyStack 삽입
        for (int i = 0; i < LANDING_Q_COUNT; i++) {
            arg[i].q = &landingQ[i];

            if (pthread_create(&tid[i], NULL, go_fuel_dec_and_check, &arg[i])) {
                printf("pthread_create failed.\n");
                return -1;
            }
        }
        // thread 종료 대기
        for (int j = 0; j < LANDING_Q_COUNT; j++) {
            if (pthread_join(tid[j], NULL)) { // Second arg: 반환하는 포인터가 저장되는 포인터 변수
                printf("pthread_join failed\n");
                return -1;
            }
        }

        //// 긴급 착륙 & 추락 한 방에 처리
        // 해당 분기를 통과하면 비어있을 경우 X
        if (emergS.size > 0) {
            // 활주로 우선순위 배열 세팅 (마지막 활주로 우선)
            int rw_priority[RUNWAY_COUNT]; // 배열: uint8_t > int 자동승격
            for (int i = 0; i < RUNWAY_COUNT; i++)
                rw_priority[i] = RUNWAY_COUNT - i - 1;

            Node *curr = pop_all_emergency(&emergS); // 스택 제거
            if (curr == NULL) {
                // 비어있을 수 없음 (emergS.size>0 이어서)
                printf("Emergency Stack is not empty. But pop_all_emergency is NULL.\n");
                return -1; // 뭔가 잘못됐으니 종료
            }

            int survived_plane_count = 0; // 최대 3

            // loop 1번으로 개선
            while (curr != NULL) {
                // 긴급 리스트 중 3개만 착륙
                if (survived_plane_count < 3) { // 최대 3번 수행

                    rw_used[rw_priority[survived_plane_count]] = 1; // 활주로 사용 명시
                    g_total_emergency_plane_count++;                // 긴급 착륙한 비행기 집계
                    l_total_landing_queue_size--;                   // 착륙했으니 감소
                    survived_plane_count++;                         // 생존했으니 증가

                    printf("[!] [EMERGENCY] ID: %d, RW: %d, Fuel: %d, Type: %d\n",
                           curr->plane.idx, rw_priority[survived_plane_count] + 1,
                           curr->plane.fuel, curr->plane.type);
                }
                // 긴급 스택이 3개 이상인 경우: 나머지 다 추락
                else {
                    // 추락한 비행기 집계
                    g_total_crashed_plane_count++;
                    l_total_landing_queue_size--; //추락했으니 감소
                    printf("[!] [CRASHED] ID: %d, Fuel: %d\n", curr->plane.idx, curr->plane.fuel);
                }
                // 정리
                Node *nextNode = curr->next; // 삭제 전 미리 저장
                free_node(curr);
                curr = nextNode; // curr == NULL 은 분기에서 처리 됨
            }
        }
        else {
            printf("Emerency Stack is empty.\n");
        }

        //// 일반 착륙 & 이륙 중 큐 길이가 긴 것 우선 처리
        //? 활주로 개수만큼 큐 길이 확인 후 이/착륙 수행 비효율 > 한 번 확인 후 같은 동작 수행이 효율적일 듯 (활주로 적을 때 유효)
        // 잔여 활주로가 있다면
        int remainRW_count; // 잔여 활주로 수
        if (remainRW_count = is_there_remain_runway(rw_used)) {

            // 빈 활주로 위치 파악
            int remainRW_idx[remainRW_count];
            for (int i = 0; i < remainRW_count; i++) {
                // 빈 활주로 idx 파악
                int idx = 0;
                for (int j = 0; j < RUNWAY_COUNT; j++) {
                    if (rw_used[j] == 1)
                        continue;
                    remainRW_idx[i++] = j;
                }
            }

            // 큐 길이 비교 후 긴 큐 소모 -> 삼항 연산자(속도 빠름)
            // 일반 이륙 수행 (이륙 큐가 더 김)
            if ((l_total_landing_queue_size < l_total_takeoff_queue_size) ? 1 : 0) {
                // 이륙 큐 중 가장 긴 큐 파악
                int takeoffQ_idx = get_longest_queue_idx(takeoffQ, TAKEOFF_Q_COUNT);
                // 한 동작이 활주로 전체 소모 -> 연산 수 감소
                for (int i = 0; i < remainRW_count; i++) {
                    Node *takeoff = dequeue(&takeoffQ[takeoffQ_idx]);
                    //! 가장 긴 큐가 잔여 활주로보다 적을 수 있음 (다른 큐로 던지기 (goto?) vs 종료)
                    if (takeoff == NULL) {
                        printf("takeoff Queue is empty.\n");
                        break;
                    }

                    l_total_takeoff_latency += (tick - takeoff->plane.entryTime); // 이륙 대기 시간 집계
                    rw_used[remainRW_idx[i]] = 1;                                 // 활주로 사용 명시
                    l_total_takeoff_queue_size--;                                 // 이륙했으니 감소
                    l_total_takeoff_plane_count++;                                // 이륙했으니 증가

                    printf("[TAKEOFF] ID: %d, RW: %d, Type: %d\n",
                           takeoff->plane.idx, remainRW_idx[i] + 1, takeoff->plane.type);

                    free_node(takeoff);
                }
            }
            // 일반 착륙 수행 (착륙 큐가 더 김)
            //! 마지막 활주로는 일반 착륙X
            else {
                // 착륙 큐 중 가장 긴 큐 파악
                int landingQ_idx = get_longest_queue_idx(landingQ, LANDING_Q_COUNT);
                // 한 동작이 활주로 전체 소모 -> 연산 수 감소
                for (int i = 0; i < remainRW_count; i++) {
                    Node *landing = dequeue(&landingQ[landingQ_idx]);
                    //! 가장 긴 큐가 잔여 활주로보다 적을 수 있음 (다른 큐로 던지기 vs 종료)
                    if (landing == NULL) {
                        printf("landing Queue is empty.\n");
                        break;
                    }

                    // 이륙 전용 활주로를 만난 경우
                    if (remainRW_idx[i] == TAKEOFF_ONLY || remainRW_idx[i] == TAKEOFF_ONLY_SECOND) {
                        // takeoff 처리 함수 실행 (코드 가독성)
                        takeoff_process(remainRW_idx[i], tick,
                                        &l_total_takeoff_latency, rw_used,
                                        &l_total_takeoff_queue_size);
                        l_total_takeoff_plane_count++;
                        continue; // break 금지
                    }

                    l_total_landing_remaining += (landing->plane.fuel / landing->plane.consume); // 남은 제한시간 집계
                    l_total_landing_latency += (tick - landing->plane.entryTime);                // 착륙 대기 시간 집계
                    rw_used[remainRW_idx[i]] = 1;                                                // 활주로 사용 명시
                    l_total_landing_queue_size--;                                                // 착륙했으니 감소
                    l_total_landing_plane_count++;                                               // 착륙했으니 증가

                    printf("[LANDING] ID: %d, RW: %d, Fuel: %d, Type: %d\n",
                           landing->plane.idx, remainRW_idx[i] + 1,
                           landing->plane.fuel, landing->plane.type);

                    free_node(landing);
                }
            }
        } // 한 단위 종료

        printf("-----------------------One loop done------------------------\n");
        // 평균 이륙 지연시간, 평균 착륙 지연시간
        if (l_total_takeoff_plane_count == 0) {
            printf("l_total_takeoff_plane_count == 0 || l_total_landing_plane_count == 0\n");
        }
        else {
            printf("[+] [Avg Takeoff Latency] %d\n",
                   l_total_takeoff_latency / l_total_takeoff_plane_count);
        }

        // 평균 착륙
        if (l_total_landing_plane_count == 0) {
            printf("l_total_landing_plane_count == 0\n");
        }
        else {
            printf("[+] [Avg Landing Latentcy] %d\n",
                   l_total_landing_latency / l_total_landing_plane_count);
            printf("[+] [Avg Remaining Time Limit] %d\n",
                   l_total_landing_remaining / l_total_landing_plane_count);
        }

        // 활주로 점유 상태
        printf("[+] [Runway Status] [");
        for (int i = 0; i < RUNWAY_COUNT; i++) {
            if (i == RUNWAY_COUNT - 1) {
                printf(" %d", rw_used[i]);
                break;
            }
            printf(" %d, ", rw_used[i]);
        }
        printf(" ]\n");

        // 큐 상태
        printf("[+] [Total Landing Queue Size] %d\n", l_total_landing_queue_size);
        printf("[+] [Total Takeoff Queue Size] %d\n", l_total_takeoff_queue_size);

    } // 시뮬레이션 종료

    printf("\n\n=============[ Simulation is done! Let's check it out! ]=============\n");
    printf("[Total Emergency Landed]: %d\n", g_total_emergency_plane_count);
    printf("[Total Crashed Planes] %d\n", g_total_crashed_plane_count);
    if (g_total_plane_count == 0) {
        printf("g_total_plane_count == 0.\n");
    }
    else {
        printf("[Avg Emergency Landed] %lf\n", ((double)g_total_emergency_plane_count / g_total_plane_count * 100.0));
        printf("[Avg Crashed Planes] %lf\n ", ((double)g_total_crashed_plane_count / g_total_plane_count * 100.0));
    }
}

// int pthread_create(pthread_t* thread,
// 										pthread_attr_t* attr, void* (*routine)(void*), void* arg);
// - thread: 생성을 성공하면 그 쓰레드의 ID가 저장된다.
// - attr: 쓰레드의 속성 객체, 기본 속성을 사용할 경우 NULL을 넣는다.
// - routine: 스레드의 실행코드 영역
// - arg: 스레드에게 전달할 인자를 담은 구조체의 포인터, Nullable.
// - 반환값: 0, 0보다 작은 값
// ------------------------------------------------------------------------------
// void pthread_exit(void* retval)
// - retval: 반환할 구조체의 포인터, Nullable
// ------------------------------------------------------------------------------
// int pthread_join(pthread_t thread, void** thread_return) : 0, 음수 에러코드
// - thread: 기다릴 스레드ID
// - thread_return: 반환하는 포인터가 저장되는 포인터 변수
// ------------------------------------------------------------------------------
// > **Lock 변수 선언 및 초기화 방법 2가지**
// 1. 전역 변수로 선언 - **`pthread_mutex_t lock;`**
// 2. 한 함수에서 선언 - **`pthread_mutex_init(&lock, NULL);
// ------------------------------------------------------------------------------
// 전역 lock 적용
// pthread_mutex_lock(&lock);
// for (i = 0; i < LOOP; i++){
//   gdata += data;
// }
// pthread_mutex_unlock(&lock);

//// 예상 문제? 큐 정렬,