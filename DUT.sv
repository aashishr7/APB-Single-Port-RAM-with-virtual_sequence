module apb_single_port_ram(
    // APB Interface signals
    input logic        PCLK,     // Clock input
    input logic        PRESETn,  // Active low reset
    input logic        PSEL,     // Peripheral select
    input logic        PENABLE,  // Enable signal
    input logic        PWRITE,   // Write control (1=Write, 0=Read)
    input logic [31:0] PADDR,    // Address bus
    input logic [31:0] PWDATA,   // Write data bus
    output logic       PREADY,   // Slave ready signal
    output logic [31:0] PRDATA,  // Read data bus
    output logic       PSLVERR   // Error response
);
    // Memory array: 1024 words, each 32-bits wide
    logic [31:0] mem [0:1023];  // 1K words of memory
    logic [31:0] tmp;           // Temporary storage for debug purposes
    
    // APB state machine definition
    typedef enum logic [1:0] {
        IDLE   = 2'b00,  // Idle state - waiting for transaction
        SETUP  = 2'b01,  // Setup state - address phase
        ACCESS = 2'b10,  // Access state - data phase
        WAIT   = 2'b11   // Wait state - for multi-cycle transactions
    } state_t;
    
    state_t present_state, next_state;  // State machine variables
    
    // State Transition Logic - Sequential part of state machine
    always_ff @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn) begin
            // Reset to IDLE state on active-low reset
            present_state <= IDLE;
        end else begin
            // Move to next state on clock edge
            present_state <= next_state;
        end
    end
    
    // Next State Logic - Combinational part of state machine
    always_comb begin
        // Default case to prevent latches
        next_state = IDLE;
        
        case (present_state)
            IDLE: begin
                // Move to SETUP when peripheral is selected
                if (PSEL) begin
                    next_state = SETUP;
                end else begin
                    next_state = IDLE;
                end
            end
            
            SETUP: begin
                // Move to ACCESS when PENABLE is asserted
                if (PENABLE) begin
                    next_state = ACCESS;
                end else begin
                    next_state = SETUP;
                end
            end
            
            ACCESS: begin
                if (PREADY) begin
                    // Transaction completed
                    if (!PSEL) begin
                        // If PSEL deasserted, go to IDLE
                        next_state = IDLE;
                    end else begin
                        // If PSEL still asserted, start new transaction
                        next_state = SETUP;
                    end
                end else begin
                    // If not ready, wait
                    next_state = WAIT;
                end
            end
            
            WAIT: begin
                // Stay in WAIT state until PREADY is asserted
                if (PREADY) begin
                    next_state = ACCESS;
                end else begin
                    next_state = WAIT;
                end
            end
            
            default: next_state = IDLE;
        endcase
    end
    
    // Memory Write Logic - Handles write operations to the memory
    always_ff @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn) begin
            // Reset temporary register on reset
            tmp <= 32'h0;
        end else if ((present_state == ACCESS) && PSEL && PENABLE && PWRITE) begin
            // Write operation: When in ACCESS state with PSEL, PENABLE active and PWRITE=1
            // Only use lower 10 bits of PADDR to address the 1024-entry memory
          mem[PADDR[9:0]] <= PWDATA;
            tmp <= PWDATA;  // Store in temp register for debug purposes
            
            // Debug messages for write operation
            //$display("\n[%0t]:WRITE:\n memory[%0h] = %0h\n PWDATA = %0h\n PWRITE = %0h", 
            //        $time, PADDR[9:0], PWDATA, PWDATA, PWRITE);                  
            //$display("[%0t] Write: memory[%0h] = %0h", $time, PADDR[9:0], PWDATA);
        end
    end
    
    // Memory Read Logic - Handles read operations from the memory
    always_ff @(posedge PCLK or negedge PRESETn) begin
        if (!PRESETn) begin
            // Reset read data on reset
            PRDATA <= 32'h0;
        end else if ((present_state == ACCESS) && PSEL && PENABLE && !PWRITE) begin
            // Read operation: When in ACCESS state with PSEL, PENABLE active and PWRITE=0
            // Only use lower 10 bits of PADDR to address the 1024-entry memory
          PRDATA <= mem[PADDR[9:0]];
            
            // Debug messages for read operation
            //$display("\n[%0t]:READ:\n memory[%0h] = %0h\n PRDATA = %0h\n PWRITE = %0h",
            //        $time, PADDR[9:0], mem[PADDR[9:0]], mem[PADDR[9:0]], PWRITE);
            //$display("tmp %0h, PRDATA %0h", tmp, mem[PADDR[9:0]]);
            //$display("[%0t] Read: PRDATA = %0h", $time, mem[PADDR[9:0]]);
        end
    end
    
    // PREADY and PSLVERR logic
    assign PSLVERR = 1'b0;  // Always indicate no error condition
    assign PREADY = 1'b1;   // Slave is always ready in this implementation
endmodule
